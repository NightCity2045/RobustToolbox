using System;
using System.Collections.Generic;
using System.Numerics;
using Robust.Shared.GameObjects;
using Robust.Shared.IoC;
using Robust.Shared.Maths;
using Robust.Shared.Spatial;
using Robust.Shared.Spatial.Interest;
using Robust.Shared.Spatial.Physics;

namespace Robust.Server.Spatial;

/// <summary>Server-side broadphase; the supplied policy must authorize every returned entity.</summary>
public interface ISpatialInterestService
{
    void Collect(EntityUid world, Box3 bounds, Func<EntityUid, bool> allowed,
        ISet<EntityUid> entities, ISet<SpatialSurfaceReference> surfaces);
}

/// <summary>Maintains derived XYZ bounds from the canonical transform hierarchy and geometry.</summary>
public sealed partial class SpatialInterestSystem : EntitySystem, ISpatialInterestService
{
    [Dependency] private SharedTransformSystem _transforms = default!;
    [Dependency] private SharedGeometrySystem _geometry = default!;

    private readonly Dictionary<EntityUid, WorldIndex> _worlds = new();
    private readonly Dictionary<EntityUid, Entry> _entries = new();
    private readonly HashSet<EntityUid> _dirty = new();
    private int _refinementFrame;

    private sealed class WorldIndex : IDisposable
    {
        public readonly SpatialIndex3<EntityUid> Entities = new();
        public readonly SpatialIndex3<SurfaceEntry> Geometry = new();
        public void Dispose() { Entities.Dispose(); Geometry.Dispose(); }
    }

    private readonly record struct SurfaceEntry(SpatialSurfaceReference Surface, EntityUid? Source);
    private sealed class Entry(EntityUid world)
    {
        public readonly EntityUid World = world;
        public SpatialGeometrySnapshot? Snapshot;
        public readonly List<SurfaceEntry> Surfaces = new();
        public readonly Dictionary<SurfaceEntry, (Vector3 Min, Vector3 Max)> LocalBounds = new();
    }

    public override void Initialize()
    {
        base.Initialize();
        EntityManager.EntityInitialized += OnEntityChanged;
        EntityManager.EntityDirtied += OnEntityChanged;
        EntityManager.EntityDeleted += OnDeleted;
        EntityManager.ComponentAdded += OnComponentAdded;
        EntityManager.ComponentRemoved += OnComponentRemoved;
        EntityManager.AfterEntityFlush += Clear;
        _transforms.OnGlobalMoveEvent += OnMove;
        SubscribeLocalEvent<WorldEditCompleted>(OnGeometryEdit);
        SubscribeLocalEvent<SpatialBodyDefinitionChanged>(OnBodyChanged);
        var query = EntityQueryEnumerator<TransformComponent>();
        while (query.MoveNext(out var uid, out _)) _dirty.Add(uid);
    }

    public override void Shutdown()
    {
        EntityManager.EntityInitialized -= OnEntityChanged;
        EntityManager.EntityDirtied -= OnEntityChanged;
        EntityManager.EntityDeleted -= OnDeleted;
        EntityManager.ComponentAdded -= OnComponentAdded;
        EntityManager.ComponentRemoved -= OnComponentRemoved;
        EntityManager.AfterEntityFlush -= Clear;
        _transforms.OnGlobalMoveEvent -= OnMove;
        Clear();
        base.Shutdown();
    }

    private void Clear()
    {
        foreach (var world in _worlds.Values) world.Dispose();
        _worlds.Clear();
        _entries.Clear();
        _dirty.Clear();
    }

    private void Mark(EntityUid uid)
    {
        if (_entries.ContainsKey(uid) || TryComp<TransformComponent>(uid, out var transform)
            && transform.MapUid is { } map && HasComp<SpatialWorldComponent>(map))
            _dirty.Add(uid);
    }
    private void OnEntityChanged(Entity<MetaDataComponent> ent) => Mark(ent.Owner);
    private void OnDeleted(Entity<MetaDataComponent> ent)
    {
        Remove(ent.Owner);
        _dirty.Remove(ent.Owner);
        if (_worlds.Remove(ent.Owner, out var world)) world.Dispose();
    }
    private void OnMove(ref MoveEvent ev) => Mark(ev.Entity.Owner);
    private void OnComponentAdded(AddedComponentEventArgs ev)
    {
        if (Relevant(ev.BaseArgs.Component)) _dirty.Add(ev.BaseArgs.Owner);
    }
    private void OnComponentRemoved(RemovedComponentEventArgs ev)
    {
        if (Relevant(ev.BaseArgs.Component)) _dirty.Add(ev.BaseArgs.Owner);
    }
    private static bool Relevant(IComponent component)
        => component is SpatialWorldComponent or SpatialFrameComponent or SpatialGeometryComponent or SpatialBodyComponent;
    private void OnBodyChanged(ref SpatialBodyDefinitionChanged ev) => _dirty.Add(ev.Entity);
    private void OnGeometryEdit(ref WorldEditCompleted ev)
    {
        if (ev.Transaction.Status != WorldEditStatus.Committed) return;
        foreach (var change in ev.Transaction.Changes) _dirty.Add(change.After.OwnerFrame);
    }

    /// <summary>Main-thread preparation before parallel PVS readers. Never mutates canonical state.</summary>
    internal void Prepare()
    {
        if (_dirty.Count == 0) return;
        var pending = new Stack<EntityUid>();
        var visited = new HashSet<EntityUid>();
        foreach (var uid in _dirty) pending.Push(uid);
        _dirty.Clear();
        while (pending.TryPop(out var uid))
        {
            if (!visited.Add(uid)) continue;
            if (!TryComp<TransformComponent>(uid, out var transform) || TerminatingOrDeleted(uid))
            {
                Remove(uid);
                continue;
            }
            if (!_entries.ContainsKey(uid)
                && (transform.MapUid is not { } map || !HasComp<SpatialWorldComponent>(map)))
                continue;
            // A parent's rotation or transfer invalidates every descendant's world bounds.
            foreach (var child in transform._children) pending.Push(child);
            UpdateEntry(uid, transform);
        }
        var removedWorlds = new List<EntityUid>();
        foreach (var (uid, world) in _worlds)
        {
            if (!TryComp<SpatialWorldComponent>(uid, out var component) || !component.Running || TerminatingOrDeleted(uid))
            {
                removedWorlds.Add(uid);
                continue;
            }
            world.Entities.Refine(_refinementFrame);
            world.Geometry.Refine(_refinementFrame);
        }
        foreach (var uid in removedWorlds)
        {
            _worlds[uid].Dispose();
            _worlds.Remove(uid);
        }
        _refinementFrame = (_refinementFrame + 1) & int.MaxValue;
    }

    private void Remove(EntityUid uid)
    {
        if (!_entries.Remove(uid, out var entry) || !_worlds.TryGetValue(entry.World, out var world)) return;
        world.Entities.Remove(uid);
        foreach (var surface in entry.Surfaces) world.Geometry.Remove(surface);
    }

    private void UpdateEntry(EntityUid uid, TransformComponent transform)
    {
        if (transform.MapUid is not { } map || !TryComp<SpatialWorldComponent>(map, out var spatialWorld)
            || !spatialWorld.Running || TerminatingOrDeleted(map))
        {
            Remove(uid);
            return;
        }
        if (!_worlds.TryGetValue(map, out var world)) _worlds.Add(map, world = new WorldIndex());
        if (_entries.TryGetValue(uid, out var entry) && entry.World != map)
        {
            Remove(uid);
            entry = null;
        }
        if (entry == null) _entries.Add(uid, entry = new Entry(map));
        var pose = _transforms.GetWorldPose(transform);
        var min = pose.Position;
        var max = min;
        if (TryComp<SpatialBodyComponent>(uid, out var body) && body.HasDefinition && !body.Definition.UseGeometry)
        {
            foreach (var collider in body.Definition.Colliders)
            {
                var half = collider.Shape switch
                {
                    SpatialBoxShape box => box.Size * 0.5f,
                    SpatialSphereShape sphere => new Vector3(sphere.Radius),
                    SpatialCapsuleShape capsule => new Vector3(capsule.Radius, capsule.Radius, capsule.Radius + capsule.Length * 0.5f),
                    _ => throw new InvalidOperationException("Unknown spatial collider bounds.")
                };
                IncludeBox(pose.Compose(collider.LocalPose), -half, half, ref min, ref max);
            }
        }
        world.Entities.SetBounds(uid, min, max);

        if (!TryComp<SpatialGeometryComponent>(uid, out var geometry) || !geometry.Running)
        {
            foreach (var surface in entry.Surfaces) world.Geometry.Remove(surface);
            entry.Surfaces.Clear();
            entry.LocalBounds.Clear();
            entry.Snapshot = null;
            return;
        }
        if (entry.Snapshot == null || entry.Snapshot.CaptureIdentity != geometry.CaptureIdentity
            || entry.Snapshot.Revision != geometry.Revision)
        {
            entry.Snapshot = _geometry.CaptureSnapshot(uid);
            entry.LocalBounds.Clear();
            foreach (var solid in entry.Snapshot.SolidParts)
            {
                var key = new SurfaceEntry(solid.Surface, solid.Entity);
                if (!entry.LocalBounds.TryGetValue(key, out var box))
                    box = (solid.Bounds.Min, solid.Bounds.Max);
                else
                    box = (Vector3.Min(box.Min, solid.Bounds.Min), Vector3.Max(box.Max, solid.Bounds.Max));
                entry.LocalBounds[key] = box;
            }
        }
        foreach (var surface in entry.Surfaces)
            if (!entry.LocalBounds.ContainsKey(surface)) world.Geometry.Remove(surface);
        entry.Surfaces.Clear();
        foreach (var (key, local) in entry.LocalBounds)
        {
            min = new Vector3(float.PositiveInfinity);
            max = new Vector3(float.NegativeInfinity);
            IncludeBox(pose, local.Min, local.Max, ref min, ref max);
            world.Geometry.SetBounds(key, min, max);
            entry.Surfaces.Add(key);
        }
    }

    private static void IncludeBox(RigidTransform3 pose, Vector3 min, Vector3 max, ref Vector3 resultMin, ref Vector3 resultMax)
    {
        for (var corner = 0; corner < 8; corner++)
        {
            var point = pose.TransformPoint(new Vector3((corner & 1) == 0 ? min.X : max.X,
                (corner & 2) == 0 ? min.Y : max.Y, (corner & 4) == 0 ? min.Z : max.Z));
            resultMin = Vector3.Min(resultMin, point);
            resultMax = Vector3.Max(resultMax, point);
        }
    }

    /// <summary>Collect on the main thread; policy must cover containers, visibility and gameplay access.</summary>
    public void Collect(EntityUid world, Box3 bounds, Func<EntityUid, bool> allowed,
        ISet<EntityUid> entities, ISet<SpatialSurfaceReference> surfaces)
    {
        Prepare();
        CollectPrepared(world, bounds, allowed, entities, surfaces);
    }

    internal bool CollectPrepared(EntityUid world, Box3 bounds, Func<EntityUid, bool> allowed,
        ISet<EntityUid> entities, ISet<SpatialSurfaceReference> surfaces)
    {
        if (!bounds.IsValid) throw new ArgumentException("Interest requires finite, positive-volume bounds.");
        ArgumentNullException.ThrowIfNull(allowed);
        ArgumentNullException.ThrowIfNull(entities);
        ArgumentNullException.ThrowIfNull(surfaces);
        if (!_worlds.TryGetValue(world, out var index)) return false;
        var complete = true;
        var candidates = new List<EntityUid>();
        index.Entities.Query(bounds.Min, bounds.Max, candidates);
        foreach (var uid in candidates)
        {
            if (allowed(uid)) entities.Add(uid);
            else if (EntityManager.TrySystem<SharedSpatialPhysicsSystem>(out var physics) && physics.HasActiveCollision(uid)) complete = false;
        }
        var geometry = new List<SurfaceEntry>();
        index.Geometry.Query(bounds.Min, bounds.Max, geometry);
        foreach (var hit in geometry)
        {
            if (!allowed(hit.Surface.OwnerFrame) || hit.Source is { } source && !allowed(source))
            {
                complete = false;
                continue;
            }
            entities.Add(hit.Surface.OwnerFrame);
            if (hit.Source is { } entity) entities.Add(entity);
            surfaces.Add(hit.Surface);
        }
        return complete;
    }
}
