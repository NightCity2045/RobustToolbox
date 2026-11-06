using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;
using Robust.Shared.GameObjects;
using Robust.Shared.Spatial;

namespace Robust.Client.Graphics.Spatial;

public readonly record struct SpatialMeshCacheLimits(int MaximumFrames, int MaximumConcurrentBuilds,
    long MaximumResidentBytes, SpatialMeshBuildLimits PerFrame)
{
    internal void Validate()
    {
        PerFrame.Validate();
        if (MaximumFrames <= 0 || MaximumConcurrentBuilds <= 0 || MaximumConcurrentBuilds > MaximumFrames
            || MaximumResidentBytes <= 0)
            throw new ArgumentOutOfRangeException(nameof(MaximumFrames), "Mesh cache budgets must be positive.");
    }
}

public enum SpatialMeshStatus : byte { Pending, Ready, BudgetExceeded, Failed }

/// <summary>
/// One world's bounded derived CPU mesh cache. Call Request only with authorized
/// snapshots, Remove on revocation/frame deletion and Dispose on world deletion.
/// All public calls belong to the owning client thread; workers see immutable
/// snapshots only. Pump never waits for a worker or touches GPU resources.
/// </summary>
/// <remarks>
/// Resident byte accounting includes inactive previous meshes used for incremental
/// jobs. In-flight memory is additionally bounded by MaximumConcurrentBuilds and
/// PerFrame.MaximumQuads/MaximumChunks; these are explicit profile inputs.
/// A newer request immediately makes old meshes unavailable to rendering.
/// </remarks>
public sealed class SpatialMeshCache : IDisposable
{
    private readonly int _thread = Environment.CurrentManagedThreadId;
    private readonly float _chunkSize;
    private readonly SpatialMeshCacheLimits _limits;
    private readonly Dictionary<EntityUid, Entry> _entries = new();
    private readonly List<Job> _jobs = new();
    private bool _disposed;
    public long ResidentBytes { get; private set; }
    public int RunningJobs => _jobs.Count;

    private sealed class Entry(SpatialGeometrySnapshot source)
    {
        public SpatialGeometrySnapshot Requested = source;
        public SpatialGeometrySnapshot? BuiltSource;
        public IReadOnlyList<SpatialChunkMesh>? Meshes;
        public SpatialMeshStatus Status;
        public Exception? Error;
        public long Bytes;
    }

    private sealed record Job(Entry Entry, SpatialGeometrySnapshot Source,
        CancellationTokenSource Cancellation, Task<IReadOnlyList<SpatialChunkMesh>> Work);

    public SpatialMeshCache(float chunkSize, SpatialMeshCacheLimits limits)
    {
        limits.Validate();
        if (!float.IsFinite(chunkSize) || chunkSize <= 0) throw new ArgumentOutOfRangeException(nameof(chunkSize));
        _chunkSize = chunkSize; _limits = limits;
    }

    public void Request(SpatialGeometrySnapshot snapshot)
    {
        CheckThread(); ArgumentNullException.ThrowIfNull(snapshot);
        if (!_entries.TryGetValue(snapshot.OwnerFrame, out var entry))
        {
            if (_entries.Count >= _limits.MaximumFrames) throw new InvalidOperationException("Spatial mesh frame budget exceeded.");
            _entries.Add(snapshot.OwnerFrame, new Entry(snapshot));
            return;
        }
        if (ReferenceEquals(entry.Requested, snapshot) && entry.Status is SpatialMeshStatus.Pending or SpatialMeshStatus.Ready) return;
        foreach (var job in _jobs.Where(j => ReferenceEquals(j.Entry, entry))) job.Cancellation.Cancel();
        entry.Requested = snapshot; entry.Status = SpatialMeshStatus.Pending; entry.Error = null;
    }

    public void Remove(EntityUid frame)
    {
        CheckThread();
        if (!_entries.Remove(frame, out var entry)) return;
        ResidentBytes -= entry.Bytes;
        foreach (var job in _jobs.Where(j => ReferenceEquals(j.Entry, entry))) job.Cancellation.Cancel();
    }

    /// <summary>Returns only the exact requested snapshot's completed meshes, never the preceding revision.</summary>
    public bool TryGet(SpatialGeometrySnapshot snapshot, out IReadOnlyList<SpatialChunkMesh>? meshes)
    {
        CheckThread(); meshes = null;
        if (!_entries.TryGetValue(snapshot.OwnerFrame, out var entry) || !ReferenceEquals(entry.Requested, snapshot)
            || entry.Status != SpatialMeshStatus.Ready) return false;
        meshes = entry.Meshes;
        return true;
    }

    public SpatialMeshStatus GetStatus(EntityUid frame, out Exception? error)
    {
        CheckThread();
        var entry = _entries[frame]; error = entry.Error; return entry.Status;
    }

    public void Pump()
    {
        CheckThread();
        for (var i = _jobs.Count - 1; i >= 0; i--)
        {
            var job = _jobs[i];
            if (!job.Work.IsCompleted) continue;
            _jobs.RemoveAt(i);
            try
            {
                var meshes = job.Work.GetAwaiter().GetResult();
                if (!Current(job)) continue;
                var bytes = meshes.Sum(m => m.EstimatedCpuBytes);
                var entry = job.Entry;
                if (bytes > _limits.MaximumResidentBytes - (ResidentBytes - entry.Bytes))
                {
                    // Drop the hidden old revision too: it cannot satisfy this request.
                    ResidentBytes -= entry.Bytes; entry.Bytes = 0; entry.Meshes = null; entry.BuiltSource = null;
                    entry.Status = SpatialMeshStatus.BudgetExceeded;
                    continue;
                }
                ResidentBytes += bytes - entry.Bytes; entry.Bytes = bytes;
                entry.Meshes = meshes; entry.BuiltSource = job.Source; entry.Status = SpatialMeshStatus.Ready;
            }
            catch (OperationCanceledException) when (job.Cancellation.IsCancellationRequested) { }
            catch (Exception error)
            {
                if (Current(job)) { job.Entry.Status = SpatialMeshStatus.Failed; job.Entry.Error = error; }
            }
            finally { job.Cancellation.Dispose(); }
        }

        foreach (var entry in _entries.Values)
        {
            if (_jobs.Count >= _limits.MaximumConcurrentBuilds) break;
            if (entry.Status != SpatialMeshStatus.Pending || _jobs.Any(j => ReferenceEquals(j.Entry, entry))) continue;
            var source = entry.Requested;
            var previous = entry.BuiltSource;
            var previousMeshes = entry.Meshes;
            var cancellation = new CancellationTokenSource();
            var token = cancellation.Token;
            var work = Task.Run(() => previous != null && previousMeshes != null
                ? SpatialChunkMeshBuilder.BuildIncremental(source, previous, previousMeshes, _chunkSize, _limits.PerFrame, token)
                : SpatialChunkMeshBuilder.Build(source, _chunkSize, _limits.PerFrame, token), token);
            _jobs.Add(new Job(entry, source, cancellation, work));
        }
    }

    private bool Current(Job job) => _entries.TryGetValue(job.Source.OwnerFrame, out var current)
        && ReferenceEquals(current, job.Entry) && ReferenceEquals(current.Requested, job.Source);

    private void CheckThread()
    {
        if (_disposed) throw new ObjectDisposedException(nameof(SpatialMeshCache));
        if (Environment.CurrentManagedThreadId != _thread) throw new InvalidOperationException("Mesh cache belongs to its creating client thread.");
    }

    public void Dispose()
    {
        if (_disposed) return;
        CheckThread(); _disposed = true;
        foreach (var job in _jobs)
        {
            job.Cancellation.Cancel();
            // Observe failures and release cancellation resources even if the world
            // is deleted before the worker reaches its next cancellation boundary.
            _ = job.Work.ContinueWith(task =>
            {
                _ = task.Exception;
                job.Cancellation.Dispose();
            }, CancellationToken.None, TaskContinuationOptions.ExecuteSynchronously, TaskScheduler.Default);
        }
        _jobs.Clear(); _entries.Clear(); ResidentBytes = 0;
    }
}
