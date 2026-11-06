using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Security.Cryptography;
using Robust.Shared.GameObjects;
using Robust.Shared.Maths;
using Robust.Shared.Timing;

namespace Robust.Shared.Spatial.Networking;

internal readonly record struct SpatialGeometryFrameManifest(NetEntity World, NetEntity Frame,
    SpatialGeometryStamp Stamp, string Hash, int Length);
internal readonly record struct SpatialGeometryCoverage(NetEntity World, Box3 Bounds);

internal sealed class SpatialGeometryManifest
{
    public const int MaximumFrames = 512;
    public const int MaximumViews = 32;
    public const int MaximumBytes = 32 * 1024 * 1024;
    public readonly ulong Sequence;
    public readonly ulong BaseSequence;
    public readonly GameTick Tick;
    public readonly IReadOnlyList<SpatialGeometryFrameManifest> Frames;
    public readonly IReadOnlyList<SpatialGeometryCoverage> Coverage;

    public SpatialGeometryManifest(ulong sequence, ulong baseSequence, GameTick tick,
        IEnumerable<SpatialGeometryFrameManifest> frames, IEnumerable<SpatialGeometryCoverage> coverage)
    {
        if (sequence == 0 || baseSequence >= sequence || tick == GameTick.Zero)
            throw new InvalidDataException("Invalid geometry baseline sequence/tick.");
        var frameArray = frames.Take(MaximumFrames + 1).ToArray();
        var views = coverage.Take(MaximumViews + 1).ToArray();
        if (frameArray.Length > MaximumFrames || views.Length > MaximumViews
            || frameArray.Select(f => f.Frame).Distinct().Count() != frameArray.Length)
            throw new InvalidDataException("Invalid geometry baseline cardinality.");
        long total = 0;
        foreach (var frame in frameArray)
        {
            if (!ValidNet(frame.World) || !ValidNet(frame.Frame) || frame.Stamp.Generation == Guid.Empty
                || frame.Length <= 0 || frame.Length > SpatialGeometryWireCodec.MaximumBytes
                || frame.Hash == null || frame.Hash.Length != 64 || !frame.Hash.All(Uri.IsHexDigit))
                throw new InvalidDataException("Invalid geometry frame manifest.");
            total += frame.Length;
        }
        if (total > MaximumBytes || views.Any(v => !ValidNet(v.World) || !v.Bounds.IsValid))
            throw new InvalidDataException("Invalid geometry baseline bounds/byte budget.");
        Sequence = sequence;
        BaseSequence = baseSequence;
        Tick = tick;
        Frames = Array.AsReadOnly(frameArray);
        Coverage = Array.AsReadOnly(views);
    }

    internal static bool ValidNet(NetEntity uid) => uid.Id > 0 && uid.Id < NetEntity.ClientEntity;
}

internal enum SpatialGeometryApplyResult : byte { Waiting, Applied, NeedBaseline }

/// <summary>
/// A bounded replica of authorized immutable geometry. Installing a complete
/// baseline is atomic across frames. It owns neither entities nor transforms,
/// and cannot declare unknown or invalidated collision coverage to be empty.
/// </summary>
internal sealed class SpatialGeometryReplica
{
    public const int FragmentSize = 16 * 1024;
    private const int MaximumSolids = 262144;
    private SpatialGeometryManifest? _active;
    private Pending? _pending;
    private Dictionary<NetEntity, CachedFrame> _frames = new();
    private readonly HashSet<NetEntity> _invalid = new();
    private ulong _highestSequence;
    private GameTick _invalidationTick;
    private bool _needsBaseline;

    internal sealed record CachedFrame(SpatialGeometryFrameManifest Description, byte[] Bytes,
        SpatialGeometrySnapshot Snapshot);
    private sealed class Incoming(SpatialGeometryFrameManifest description, byte[] bytes, bool cached)
    {
        public readonly SpatialGeometryFrameManifest Description = description;
        public readonly byte[] Bytes = bytes;
        public readonly bool[] Received = new bool[(bytes.Length + FragmentSize - 1) / FragmentSize];
        public int Remaining = cached ? 0 : (bytes.Length + FragmentSize - 1) / FragmentSize;
        public readonly bool Cached = cached;
        public NetEntity[]? References = cached ? SpatialGeometryWireCodec.ReadReferences(bytes) : null;
    }
    private sealed class Pending(SpatialGeometryManifest manifest)
    {
        public readonly SpatialGeometryManifest Manifest = manifest;
        public readonly Dictionary<NetEntity, Incoming> Frames = new();
    }

    public ulong Sequence => _active?.Sequence ?? 0;
    public bool NeedsBaseline => _needsBaseline;
    public bool HasPending => _pending != null;
    public IEnumerable<NetEntity> KnownFrames => _frames.Keys;

    public bool Begin(SpatialGeometryManifest manifest)
    {
        if (manifest.Sequence <= _highestSequence) return false;
        _highestSequence = manifest.Sequence;
        if (manifest.BaseSequence != 0 && manifest.BaseSequence != Sequence)
        {
            RequireBaseline();
            return false;
        }
        var pending = new Pending(manifest);
        foreach (var description in manifest.Frames)
        {
            _frames.TryGetValue(description.Frame, out var old);
            // A full baseline transfers every byte again. Acknowledging cached
            // bytes early would strand the sender's fragment progress window.
            var cached = manifest.BaseSequence != 0 && old?.Description == description;
            pending.Frames.Add(description.Frame,
                new Incoming(description, cached ? old!.Bytes : new byte[description.Length], cached));
        }
        foreach (var (frame, old) in _frames)
            if (!pending.Frames.TryGetValue(frame, out var incoming) || incoming.Description != old.Description)
                _invalid.Add(frame);
        _pending = pending;
        _needsBaseline = false;
        return true;
    }

    public bool AddFragment(ulong sequence, NetEntity frame, int offset, ReadOnlySpan<byte> data)
    {
        if (_pending is not { } pending || sequence != pending.Manifest.Sequence) return false;
        if (!pending.Frames.TryGetValue(frame, out var incoming) || offset < 0 || offset % FragmentSize != 0
            || offset >= incoming.Bytes.Length || data.Length != Math.Min(FragmentSize, incoming.Bytes.Length - offset))
            throw Reject("Invalid geometry fragment bounds.");
        var part = offset / FragmentSize;
        var destination = incoming.Bytes.AsSpan(offset, data.Length);
        if (incoming.Cached || incoming.Received[part])
        {
            if (!data.SequenceEqual(destination)) throw Reject("Conflicting duplicate geometry fragment.");
            return true;
        }
        data.CopyTo(destination);
        incoming.Received[part] = true;
        incoming.Remaining--;
        if (incoming.Remaining == 0)
        {
            if (Convert.ToHexString(SHA256.HashData(incoming.Bytes)) != incoming.Description.Hash)
                throw Reject("Geometry payload hash mismatch.");
            try { incoming.References = SpatialGeometryWireCodec.ReadReferences(incoming.Bytes); }
            catch { RequireBaseline(); throw; }
        }
        return true;
    }

    /// <param name="resolve">Read-only lookup of initialized entities in the requested world; null means wait.</param>
    /// <param name="expected">The geometry stamp received in the frame's authoritative entity state.</param>
    public SpatialGeometryApplyResult TryApply(GameTick lastAuthoritativeTick,
        Func<NetEntity, NetEntity, EntityUid?> resolve, Func<EntityUid, SpatialGeometryStamp?> expected)
    {
        try { return TryApplyCore(lastAuthoritativeTick, resolve, expected); }
        catch { RequireBaseline(); throw; }
    }

    private SpatialGeometryApplyResult TryApplyCore(GameTick lastAuthoritativeTick,
        Func<NetEntity, NetEntity, EntityUid?> resolve, Func<EntityUid, SpatialGeometryStamp?> expected)
    {
        if (_needsBaseline) return SpatialGeometryApplyResult.NeedBaseline;
        if (_invalid.Count != 0 && (_pending == null || _pending.Manifest.Tick < _invalidationTick))
        {
            RequireBaseline();
            return SpatialGeometryApplyResult.NeedBaseline;
        }
        if (_pending is not { } pending || lastAuthoritativeTick < pending.Manifest.Tick
            || pending.Frames.Values.Any(f => f.Remaining != 0)) return SpatialGeometryApplyResult.Waiting;
        var next = new Dictionary<NetEntity, CachedFrame>();
        var solids = 0;
        // Resolve every dependency before any expensive geometry partitioning.
        // Waiting for a late parent must not rebuild earlier frames every tick.
        foreach (var incoming in pending.Frames.Values)
            foreach (var reference in incoming.References!)
                if (resolve(incoming.Description.World, reference) == null) return SpatialGeometryApplyResult.Waiting;
        foreach (var incoming in pending.Frames.Values)
        {
            var description = incoming.Description;
            if (resolve(description.World, description.Frame) is not { } frame) return SpatialGeometryApplyResult.Waiting;
            if (expected(frame) != description.Stamp)
            {
                RequireBaseline();
                return SpatialGeometryApplyResult.NeedBaseline;
            }
            SpatialGeometrySnapshot snapshot;
            if (incoming.Cached && _frames.TryGetValue(description.Frame, out var old))
                snapshot = old.Snapshot;
            else
            {
                try
                {
                    snapshot = SpatialGeometryWireCodec.Decode(incoming.Bytes,
                        net => resolve(description.World, net) ?? throw new MissingGeometryReference());
                }
                catch (MissingGeometryReference) { return SpatialGeometryApplyResult.Waiting; }
            }
            if (snapshot.OwnerFrame != frame || snapshot.Revision != description.Stamp.Revision)
                throw new InvalidDataException("Geometry payload differs from its frame manifest.");
            solids = checked(solids + snapshot.SolidParts.Count);
            if (solids > MaximumSolids) throw new InvalidDataException("Geometry replica solid budget exceeded.");
            next.Add(description.Frame, new CachedFrame(description, incoming.Bytes, snapshot));
        }
        // Coverage may refer to an empty spatial world with no geometry frames.
        foreach (var view in pending.Manifest.Coverage)
            if (resolve(view.World, view.World) == null) return SpatialGeometryApplyResult.Waiting;
        _frames = next;
        _active = pending.Manifest;
        _invalid.Clear();
        _pending = null;
        return SpatialGeometryApplyResult.Applied;
    }

    public void Invalidate(NetEntity frame, GameTick tick = default)
    {
        // A newly replicated frame can introduce collision inside coverage that
        // was previously empty. It need not already exist in the geometry cache.
        _invalid.Add(frame);
        if (tick > _invalidationTick) _invalidationTick = tick;
    }

    public void InvalidateKnown(NetEntity frame, GameTick tick)
    {
        if (_frames.ContainsKey(frame) || _pending?.Frames.ContainsKey(frame) == true) Invalidate(frame, tick);
    }

    public bool TryGetSnapshot(NetEntity frame, out SpatialGeometrySnapshot? snapshot)
    {
        if (!_needsBaseline && !_invalid.Contains(frame) && _frames.TryGetValue(frame, out var cached))
        {
            snapshot = cached.Snapshot;
            return true;
        }
        snapshot = null;
        return false;
    }

    public bool IsCovered(NetEntity world, Box3 bounds)
    {
        if (!bounds.IsValid || _needsBaseline || _pending != null || _invalid.Count != 0 || _active == null) return false;
        foreach (var coverage in _active.Coverage)
            if (coverage.World == world && coverage.Bounds.Contains(bounds.Min) && coverage.Bounds.Contains(bounds.Max)) return true;
        return false;
    }

    public void RequireBaseline()
    {
        _pending = null;
        _needsBaseline = true;
    }

    private InvalidDataException Reject(string message)
    {
        RequireBaseline();
        return new InvalidDataException(message);
    }

    public void Clear()
    {
        _pending = null;
        _active = null;
        _frames.Clear();
        _invalid.Clear();
        _highestSequence = 0;
        _invalidationTick = default;
        _needsBaseline = false;
    }

    private sealed class MissingGeometryReference : Exception;
}
