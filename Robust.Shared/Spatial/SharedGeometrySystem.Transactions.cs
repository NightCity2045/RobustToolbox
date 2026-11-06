using System;
using System.Collections.Generic;
using System.Linq;
using Robust.Shared.GameObjects;
using Robust.Shared.IoC;
using Robust.Shared.Network;
using Robust.Shared.Spatial.Physics;

namespace Robust.Shared.Spatial;

public sealed partial class SharedGeometrySystem
{
    [Dependency] private INetManager _net = default!;
    private readonly HashSet<WorldEditTransaction> _transactions = new();
    private readonly Queue<WorldEditTransaction> _pending = new();
    private readonly List<WorldEditTransaction> _notifications = new();
    private bool _committing;

    private void InitializeTransactions()
    {
        SubscribeLocalEvent<SpatialGeometryComponent, ComponentShutdown>(OnGeometryShutdown);
        SubscribeLocalEvent<SpatialWorldComponent, SpatialWorldStopping>(OnWorldShutdown);
    }

    public override void Shutdown()
    {
        foreach (var transaction in _transactions.ToArray())
            Finish(transaction, WorldEditStatus.Cancelled, "Geometry service shut down.");
        _pending.Clear();
        _notifications.Clear();
        base.Shutdown();
    }

    /// <summary>Prepare detached definitions and collision shapes. No live geometry/body is changed.</summary>
    public WorldEditTransaction PrepareTransaction(IEnumerable<SpatialGeometryEdit> edits)
    {
        RequireAuthority();
        var input = edits.ToArray();
        if (input.Length == 0 || input.Select(e => e.Frame).Distinct().Count() != input.Length)
            throw new ArgumentException("An edit requires at least one frame, without duplicate frame entries.");
        var changes = new List<SpatialGeometryChange>();
        var geometry = new List<SpatialGeometryComponent>();
        var frames = new List<SpatialFrameComponent>();
        var prepared = new List<SharedSpatialPhysicsSystem.PreparedGeometryBody>();
        EntityUid? world = null;
        try
        {
            foreach (var edit in input)
            {
                if (TerminatingOrDeleted(edit.Frame) || !TryComp<SpatialGeometryComponent>(edit.Frame, out var component)
                    || !component.Running || component.Revision != edit.ExpectedRevision
                    || !ReferenceEquals(component.CaptureIdentity, edit.ExpectedIdentity))
                    throw new InvalidOperationException("Geometry frame is unavailable or its expected revision is stale.");
                var owner = EntityManager.System<SharedSpatialSystem>().GetWorld(edit.Frame).Owner;
                if (world != null && world != owner)
                    throw new InvalidOperationException("One structural transaction belongs to one spatial world.");
                world = owner;
                ValidateSources(edit.Draft, owner);
                var change = GeometryChangeBuilder.Build(CaptureSnapshot(edit.Frame), edit.Draft);
                var body = EntityManager.System<SharedSpatialPhysicsSystem>().PrepareGeometryBody(edit.Frame, owner, change.After,
                    change.Surfaces.Count == 0);
                if (body != null) prepared.Add(body);
                changes.Add(change);
                geometry.Add(component);
                frames.Add(Comp<SpatialFrameComponent>(edit.Frame));
            }
            TryComp<SpatialPhysicsWorldComponent>(world!.Value, out var profile);
            var transaction = new WorldEditTransaction(this, world.Value, changes.ToArray(), geometry.ToArray(),
                frames.ToArray(), prepared.ToArray(), profile);
            _transactions.Add(transaction);
            return transaction;
        }
        catch
        {
            foreach (var body in prepared) body.Dispose();
            throw;
        }
    }

    /// <summary>Request a commit at the next pre-physics boundary; gameplay validation/reservations belong to content.</summary>
    public void QueueTransaction(WorldEditTransaction transaction)
    {
        RequireAuthority();
        if (transaction.Service != this || transaction.Status != WorldEditStatus.Prepared || !_transactions.Contains(transaction))
            throw new InvalidOperationException("Only this service's prepared transaction can be queued once.");
        transaction.Status = WorldEditStatus.Queued;
        _pending.Enqueue(transaction);
    }

    public void CancelTransaction(WorldEditTransaction transaction)
    {
        if (transaction.Service != this)
            throw new InvalidOperationException("Transaction belongs to another geometry service.");
        if (transaction.Status is WorldEditStatus.Prepared or WorldEditStatus.Queued)
            Finish(transaction, WorldEditStatus.Cancelled, null);
    }

    /// <summary>Acceptance guard for a detached background result; stale/deleted geometry cannot be reinstalled.</summary>
    public bool IsSnapshotCurrent(SpatialGeometrySnapshot snapshot)
        => IsRevisionCurrent(snapshot.OwnerFrame, snapshot.Revision)
            && ReferenceEquals(Comp<SpatialGeometryComponent>(snapshot.OwnerFrame).CaptureIdentity, snapshot.CaptureIdentity);

    private bool IsRevisionCurrent(EntityUid frame, ulong revision)
        => !TerminatingOrDeleted(frame) && TryComp<SpatialGeometryComponent>(frame, out var geometry)
            && geometry.Running && geometry.Revision == revision;

    internal void CommitPending()
    {
        if (!_net.IsServer)
            return;
        if (_committing)
            throw new InvalidOperationException("Structural commit boundaries cannot be reentered.");
        _committing = true;
        try
        {
            var count = _pending.Count;
            for (var i = 0; i < count; i++)
            {
                var transaction = _pending.Dequeue();
                if (transaction.Status != WorldEditStatus.Queued)
                    continue;
                try { Commit(transaction); }
                catch (Exception error) when (error is InvalidOperationException or ArgumentException or KeyNotFoundException)
                {
                    Finish(transaction, WorldEditStatus.Rejected, error.Message);
                }
            }
            // A callback can enqueue another edit, but it cannot apply that edit
            // recursively inside this batch or expose partially installed frames.
            var notifications = _notifications.ToArray();
            _notifications.Clear();
            foreach (var transaction in notifications)
            {
                var ev = new WorldEditCompleted(transaction);
                RaiseLocalEvent(ref ev);
            }
        }
        finally { _committing = false; }
    }

    private void Commit(WorldEditTransaction transaction)
    {
        if (TerminatingOrDeleted(transaction.World) || !HasComp<SpatialWorldComponent>(transaction.World))
            throw new InvalidOperationException("The edited spatial world no longer exists.");
        TryComp<SpatialPhysicsWorldComponent>(transaction.World, out var profile);
        if (!ReferenceEquals(profile, transaction.PhysicsProfile))
            throw new InvalidOperationException("The physical world profile changed after preparation.");
        for (var i = 0; i < transaction.Changes.Count; i++)
        {
            var change = transaction.Changes[i];
            var uid = change.Before.OwnerFrame;
            if (!IsRevisionCurrent(uid, change.Before.Revision)
                || !ReferenceEquals(Comp<SpatialGeometryComponent>(uid), transaction.Components[i])
                || !ReferenceEquals(Comp<SpatialFrameComponent>(uid), transaction.Frames[i])
                || transaction.Frames[i].Layers.Count != change.Before.Layers.Count
                || change.Before.Layers.Any(p => !transaction.Frames[i].Layers.TryGetValue(p.Key, out var layer) || layer != p.Value)
                || EntityManager.System<SharedSpatialSystem>().GetWorld(uid).Owner != transaction.World)
                throw new InvalidOperationException("A frame changed or moved after transaction preparation.");
            ValidateSources(change.After, transaction.World);
        }
        foreach (var body in transaction.Physics) body.Validate();
        // Allocate replacement canonical collections before inserting/removing
        // bodies. No gameplay event or worker result runs during installation.
        var layers = transaction.Changes.Select(c => new Dictionary<LayerId, LayerDefinition>(c.After.Layers)).ToArray();
        var boundaries = transaction.Changes.Select(c => c.After.Boundaries.Values.ToList()).ToArray();
        var slabs = transaction.Changes.Select(c => c.After.Slabs.Values.ToList()).ToArray();
        var chunks = transaction.Changes.Select(c => c.After.TileChunks.Values.ToList()).ToArray();
        foreach (var body in transaction.Physics) body.CommitNewBody();
        for (var i = 0; i < transaction.Changes.Count; i++)
        {
            var component = transaction.Components[i];
            transaction.Frames[i]._layers = layers[i];
            component._boundaries = boundaries[i];
            component._slabs = slabs[i];
            component._tileChunks = chunks[i];
            component._revision = transaction.Changes[i].After.Revision;
        }
        foreach (var body in transaction.Physics) body.Install();
        for (var i = 0; i < transaction.Changes.Count; i++)
            Dirty(transaction.Changes[i].After.OwnerFrame, transaction.Frames[i]);
        Finish(transaction, WorldEditStatus.Committed, null);
    }

    private void Finish(WorldEditTransaction transaction, WorldEditStatus status, string? failure)
    {
        var queued = transaction.Status == WorldEditStatus.Queued;
        transaction.Status = status;
        transaction.Failure = failure;
        foreach (var body in transaction.Physics) body.Dispose();
        _transactions.Remove(transaction);
        if (queued && !transaction.Notified)
        {
            transaction.Notified = true;
            _notifications.Add(transaction);
        }
    }

    private void ValidateSources(SpatialGeometrySnapshot snapshot, EntityUid world)
    {
        foreach (var entity in snapshot.Boundaries.Values.Select(b => b.Entity).Concat(snapshot.Slabs.Values.Select(s => s.Entity)))
            if (entity is { } uid && (TerminatingOrDeleted(uid)
                || EntityManager.System<SharedSpatialSystem>().GetWorld(uid).Owner != world))
                throw new InvalidOperationException("A geometry source entity must belong to the same live spatial world.");
    }

    private void OnGeometryShutdown(EntityUid uid, SpatialGeometryComponent component, ComponentShutdown args)
    {
        foreach (var transaction in _transactions.Where(t => t.Changes.Any(c => c.Before.OwnerFrame == uid)).ToArray())
            Finish(transaction, WorldEditStatus.Rejected, "An edited geometry frame was removed.");
        var ev = new SpatialGeometryStopping();
        RaiseLocalEvent(uid, ref ev);
    }

    private void OnWorldShutdown(EntityUid uid, SpatialWorldComponent component, ref SpatialWorldStopping args)
    {
        foreach (var transaction in _transactions.Where(t => t.World == uid).ToArray())
            Finish(transaction, WorldEditStatus.Rejected, "The edited spatial world was removed.");
    }

    private void RequireAuthority()
    {
        if (!_net.IsServer)
            throw new InvalidOperationException("Structural transactions require server authority.");
    }
}
