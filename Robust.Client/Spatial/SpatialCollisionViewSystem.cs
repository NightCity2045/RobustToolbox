using System;
using System.Collections.Generic;
using System.Diagnostics.CodeAnalysis;
using System.Numerics;
using Robust.Client.GameStates;
using Robust.Shared.GameObjects;
using Robust.Shared.IoC;
using Robust.Shared.Maths;
using Robust.Shared.Network;
using Robust.Shared.Spatial;
using Robust.Shared.Spatial.Physics;
using Robust.Shared.Timing;

namespace Robust.Client.Spatial;

public readonly record struct SpatialCollisionViewLimits(int MaximumEntityVisits, int MaximumEntities, int MaximumShapes);

/// <summary>
/// Read-only client collision views derived from complete authoritative states
/// and matching authorized geometry. They neither step physics nor grant PVS
/// subscriptions. Missing geometry prevents queries rather than acting as air.
/// </summary>
public sealed partial class SpatialCollisionViewSystem : EntitySystem
{
    [Dependency] private IClientGameStateManager _states = default!;
    [Dependency] private IClientNetManager _net = default!;
    [Dependency] private SharedTransformSystem _transforms = default!;
    [Dependency] private SpatialGeometryStreamingSystem _geometry = default!;
    private readonly HashSet<SpatialCollisionView> _views = new();

    internal sealed record CapturedEntity(EntityUid Parent, RigidTransform3 Pose, bool Contained,
        SpatialBodyDefinition? Body, ulong DefinitionRevision, SpatialGeometryStamp? Geometry);
    internal sealed record Capture(GameTick Tick, Dictionary<EntityUid, CapturedEntity> Entities,
        Dictionary<EntityUid, RigidTransform3> Poses);

    public override void Initialize()
    {
        base.Initialize();
        _states.GameStateApplied += OnApplied;
        _net.Disconnect += OnDisconnect;
        EntityManager.EntityDeleted += OnDeleted;
        EntityManager.AfterEntityFlush += Clear;
        _transforms.OnGlobalMoveEvent += OnMove;
    }

    public override void Shutdown()
    {
        _states.GameStateApplied -= OnApplied;
        _net.Disconnect -= OnDisconnect;
        EntityManager.EntityDeleted -= OnDeleted;
        EntityManager.AfterEntityFlush -= Clear;
        _transforms.OnGlobalMoveEvent -= OnMove;
        Clear(); base.Shutdown();
    }

    public SpatialCollisionView CreateView(EntityUid world, Box3 bounds, SpatialCollisionViewLimits limits)
    {
        if (!world.IsValid() || !bounds.IsValid || limits.MaximumEntityVisits <= 0
            || limits.MaximumEntities <= 0 || limits.MaximumShapes <= 0)
            throw new ArgumentException("A collision view requires a valid world, finite bounds and positive budgets.");
        var view = new SpatialCollisionView(this, world, bounds, limits);
        _views.Add(view); return view;
    }

    internal void Release(SpatialCollisionView view) => _views.Remove(view);
    private void Clear() { foreach (var view in new List<SpatialCollisionView>(_views)) view.Dispose(); }
    private void OnDisconnect(object? sender, NetDisconnectedArgs args) => Clear();
    private void OnDeleted(Entity<MetaDataComponent> entity)
    {
        foreach (var view in new List<SpatialCollisionView>(_views))
            if (view.World == entity.Owner) view.Dispose();
            else if (view.Current?.Entities.ContainsKey(entity.Owner) == true) view.Invalidate();
    }
    private void OnMove(ref MoveEvent args)
    {
        if (!args.ParentChanged) return;
        foreach (var view in _views)
            if (view.Current?.Entities.ContainsKey(args.Sender) == true && args.Component.MapUid != view.World) view.Invalidate();
    }

    private void OnApplied(GameStateAppliedArgs args)
    {
        foreach (var view in _views)
        {
            view.Invalidate();
            if (!HasComp<SpatialWorldComponent>(view.World)) continue;
            var captured = new Dictionary<EntityUid, CapturedEntity>();
            var poses = new Dictionary<EntityUid, RigidTransform3>();
            var visits = 0;
            var shapes = 0;
            var valid = true;
            var query = EntityManager.AllEntityQueryEnumerator<TransformComponent, MetaDataComponent>();
            while (query.MoveNext(out var uid, out var transform, out var metadata))
            {
                if (++visits > view.Limits.MaximumEntityVisits) { view.BudgetExceeded = true; valid = false; break; }
                if (transform.MapUid != view.World || metadata.NetEntity.IsClientSide() || !metadata.NetEntity.IsValid()
                    || metadata.EntityLifeStage < EntityLifeStage.Initialized || metadata.EntityLifeStage >= EntityLifeStage.Terminating
                    || (metadata.Flags & MetaDataFlags.Detached) != 0) continue;
                if (captured.Count >= view.Limits.MaximumEntities) { view.BudgetExceeded = true; valid = false; break; }
                TryComp<SpatialBodyComponent>(uid, out var body);
                if (body != null && !body.HasDefinition) { valid = false; break; }
                if (body != null)
                {
                    if (body.Definition.Colliders.Count > view.Limits.MaximumShapes - shapes)
                    { view.BudgetExceeded = true; valid = false; break; }
                    shapes += body.Definition.Colliders.Count;
                }
                var pose = _transforms.GetWorldPose(transform);
                captured.Add(uid, new CapturedEntity(transform.ParentUid, pose,
                    (metadata.Flags & MetaDataFlags.InContainer) != 0, body?.Definition.Copy(), body?.DefinitionRevision ?? 0,
                    TryComp<SpatialFrameComponent>(uid, out var frame) ? frame.ReceivedGeometryStamp : null));
                poses.Add(uid, pose);
            }
            if (!valid || !captured.ContainsKey(view.World)) continue;
            foreach (var (uid, entity) in captured)
                if (uid != view.World && !captured.ContainsKey(entity.Parent)) { valid = false; break; }
            if (valid) view.Current = new Capture(args.AppliedState.ToSequence, captured, poses);
        }
    }

    internal bool IsCurrent(SpatialCollisionView view, Capture capture, ulong sequence)
        => !view.IsDisposed && ReferenceEquals(view.Current, capture) && _geometry.Sequence == sequence
            && _geometry.IsReady(view.World, view.Bounds);

    internal SpatialClientCollisionScene? Build(SpatialCollisionView view)
    {
        var capture = view.Current;
        if (capture == null || !_geometry.IsReady(view.World, view.Bounds)) return null;
        var sequence = _geometry.Sequence;
        var snapshots = new Dictionary<EntityUid, SpatialGeometrySnapshot>();
        foreach (var snapshot in _geometry.CaptureSnapshots(view.World))
        {
            if (!capture.Entities.TryGetValue(snapshot.OwnerFrame, out var entity)
                || entity.Geometry is not { } stamp || stamp.Revision != snapshot.Revision
                || !TryComp<SpatialFrameComponent>(snapshot.OwnerFrame, out var frame) || frame.ReceivedGeometryStamp != stamp)
                return null;
            snapshots.Add(snapshot.OwnerFrame, snapshot);
        }
        var bodies = new List<(SpatialBackendBodyDefinition Definition, RigidTransform3 Pose)>();
        var count = 0;
        var contained = new Dictionary<EntityUid, bool> { [view.World] = false };
        foreach (var (uid, entity) in capture.Entities)
        {
            if (InContainer(uid)) continue;
            var parts = new List<SpatialBackendPart>();
            if (entity.Body is { UseGeometry: false } body)
            {
                foreach (var collider in body.Colliders)
                    if (!Add(new SpatialBackendPart(collider.Shape.ToBackend(), collider.LocalPose, uid,
                            Revision: entity.DefinitionRevision, MassWeight: collider.MassWeight))) return null;
            }
            else if (snapshots.TryGetValue(uid, out var geometry))
            {
                foreach (var solid in geometry.SolidParts)
                {
                    if (solid.Entity is { } source && !capture.Entities.ContainsKey(source)) return null;
                    if (!Add(new SpatialBackendPart(new SpatialBox(solid.Bounds.Size),
                            new RigidTransform3(solid.Bounds.Center, Quaternion.Identity), solid.Entity ?? uid,
                            solid.Surface, solid.Revision, MaterialProfile: solid.MaterialProfile))) return null;
                }
            }
            if (parts.Count != 0)
                bodies.Add((new SpatialBackendBodyDefinition(SpatialBackendMotion.Static, parts,
                    collisionLayer: entity.Body?.CollisionLayer ?? 1, collisionMask: entity.Body?.CollisionMask ?? uint.MaxValue), entity.Pose));

            bool Add(SpatialBackendPart part)
            {
                // Keep every supplied part. No guessed broadphase omission may turn
                // a rotated or compound obstacle into empty space at the query edge.
                if (++count > view.Limits.MaximumShapes) { view.BudgetExceeded = true; return false; }
                parts.Add(part); return true;
            }
        }
        if (!IsCurrent(view, capture, sequence)) return null;
        return new SpatialClientCollisionScene(view.World, capture.Tick, sequence, view.Bounds, capture.Poses, snapshots,
            bodies, () => IsCurrent(view, capture, sequence));

        bool InContainer(EntityUid uid)
        {
            var path = new List<EntityUid>();
            var cursor = uid;
            while (!contained.ContainsKey(cursor))
            {
                path.Add(cursor); cursor = capture.Entities[cursor].Parent;
            }
            var value = contained[cursor];
            for (var i = path.Count - 1; i >= 0; i--)
            {
                value |= capture.Entities[path[i]].Contained; contained.Add(path[i], value);
            }
            return contained[uid];
        }
    }
}

/// <summary>Explicit client view. Content owns bounds and lifetime, never the scene's colliders.</summary>
public sealed class SpatialCollisionView : IDisposable
{
    private readonly SpatialCollisionViewSystem _owner;
    private readonly int _thread = Environment.CurrentManagedThreadId;
    private SpatialClientCollisionScene? _scene;
    internal SpatialCollisionViewSystem.Capture? Current;
    public EntityUid World { get; }
    public Box3 Bounds { get; private set; }
    public SpatialCollisionViewLimits Limits { get; }
    public bool IsDisposed { get; private set; }
    public bool BudgetExceeded { get; internal set; }
    internal SpatialCollisionView(SpatialCollisionViewSystem owner, EntityUid world, Box3 bounds, SpatialCollisionViewLimits limits)
    { _owner = owner; World = world; Bounds = bounds; Limits = limits; }

    public void SetBounds(Box3 bounds)
    {
        CheckThread(); ObjectDisposedException.ThrowIf(IsDisposed, this);
        if (!bounds.IsValid) throw new ArgumentException("Invalid collision view bounds.");
        if (Bounds == bounds) return;
        Invalidate(); Bounds = bounds;
    }

    public bool TryGetScene([NotNullWhen(true)] out SpatialClientCollisionScene? scene)
    {
        CheckThread(); scene = null;
        if (IsDisposed) return false;
        if (_scene?.IsAvailable != true) { _scene?.Dispose(); _scene = _owner.Build(this); }
        scene = _scene; return scene != null;
    }

    internal void Invalidate()
    { CheckThread(); _scene?.Dispose(); _scene = null; Current = null; BudgetExceeded = false; }
    public void Dispose()
    {
        CheckThread();
        if (IsDisposed) return;
        Invalidate(); IsDisposed = true; _owner.Release(this);
    }
    private void CheckThread()
    {
        if (_thread != Environment.CurrentManagedThreadId) throw new InvalidOperationException("Collision views require their client thread.");
    }
}

/// <summary>
/// Authorized client collision at a complete state tick, with matching geometry.
/// A read-only snapshot, not a server hit decision or a local prediction step.
/// Queries outside its certified bounds or after invalidation throw.
/// </summary>
public sealed class SpatialClientCollisionScene : ISpatialQueryService, IDisposable
{
    private readonly SpatialSnapshotQueries _queries;
    private readonly BepuSpatialPhysicsBackend _backend;
    private readonly Func<bool> _current;
    private readonly int _thread = Environment.CurrentManagedThreadId;
    private bool _querying;
    private bool _disposed;
    public EntityUid World { get; }
    public GameTick Tick { get; }
    public ulong GeometrySequence { get; }
    public bool IsAvailable => !_disposed && _current();

    internal SpatialClientCollisionScene(EntityUid world, GameTick tick, ulong sequence, Box3 bounds,
        Dictionary<EntityUid, RigidTransform3> poses, Dictionary<EntityUid, SpatialGeometrySnapshot> geometry,
        List<(SpatialBackendBodyDefinition Definition, RigidTransform3 Pose)> bodies, Func<bool> current)
    {
        World = world; Tick = tick; GeometrySequence = sequence; _current = current;
        _backend = new BepuSpatialPhysicsBackend(Vector3.Zero);
        try
        {
            foreach (var (definition, pose) in bodies) _backend.AddBody(definition, new SpatialBackendState(pose, Vector3.Zero, Vector3.Zero));
            _queries = new SpatialSnapshotQueries(world, poses, geometry, () => IsAvailable, Begin, End, bounds);
        }
        catch { _backend.Dispose(); throw; }
    }

    private ISpatialPhysicsBackend Begin()
    {
        CheckThread();
        if (_querying) throw new InvalidOperationException("A collision query policy cannot query recursively.");
        if (!IsAvailable) throw new InvalidOperationException("The collision snapshot is unavailable.");
        _querying = true; return _backend;
    }
    private void End()
    {
        _querying = false;
        if (_disposed) _backend.Dispose();
        if (!IsAvailable) throw new InvalidOperationException("The collision snapshot was invalidated during its query.");
    }
    public void Dispose()
    {
        CheckThread();
        if (_disposed) return;
        _disposed = true;
        if (!_querying) _backend.Dispose();
    }
    private void CheckThread()
    {
        if (_thread != Environment.CurrentManagedThreadId) throw new InvalidOperationException("Collision queries require their client thread.");
    }
    public SpatialHit? RayCast(SpatialQueryContext context, Vector3 origin, Vector3 direction, float distance)
        => _queries.RayCast(context, origin, direction, distance);
    public SpatialHit? FindSurface(SpatialQueryContext context, Vector3 origin, Vector3 direction, float distance)
        => _queries.FindSurface(context, origin, direction, distance);
    public IReadOnlyList<SpatialHit> ShapeCast(SpatialQueryContext context, SpatialShape shape, RigidTransform3 pose,
        Vector3 translation, Vector3 angularDisplacement) => _queries.ShapeCast(context, shape, pose, translation, angularDisplacement);
    public IReadOnlyList<SpatialHit> Overlap(SpatialQueryContext context, SpatialShape shape, RigidTransform3 pose)
        => _queries.Overlap(context, shape, pose);
    public bool IsVolumeFree(SpatialQueryContext context, SpatialShape shape, RigidTransform3 pose)
        => _queries.IsVolumeFree(context, shape, pose);
    public IReadOnlyList<SpatialOverlap> OverlapContacts(SpatialQueryContext context, SpatialShape shape, RigidTransform3 pose)
        => _queries.OverlapContacts(context, shape, pose);
}
