using System;
using System.Collections.Generic;
using System.Diagnostics.CodeAnalysis;
using System.Linq;
using System.Numerics;
using Robust.Shared.GameObjects;
using Robust.Shared.IoC;
using Robust.Shared.Maths;
using Robust.Shared.Network;
using Robust.Shared.Timing;

namespace Robust.Shared.Spatial.Physics;

/// <summary>
/// Content chooses the lag-compensation window and aggregate sample budgets.
/// Engine history is disabled until explicitly configured for a physical world.
/// Budgets count retained samples, including shared shapes conservatively.
/// </summary>
public readonly record struct SpatialHistorySettings(uint RetainedTicks, int MaximumPoseSamples, int MaximumShapeSamples)
{
    public void Validate()
    {
        if (RetainedTicks == 0 || RetainedTicks > 1024 || MaximumPoseSamples <= 0 || MaximumShapeSamples <= 0)
            throw new ArgumentOutOfRangeException(nameof(SpatialHistorySettings));
    }
}

[ByRefEvent]
public readonly record struct SpatialHistoryGap(EntityUid World, GameTick Tick, string Reason);

/// <summary>
/// Bounded coherent history at the completed physical tick. Queries never mutate
/// current transforms, bodies, geometry or gameplay. Missing ticks remain missing;
/// no interpolation, nearest-tick fallback or gameplay hit policy is implied.
/// </summary>
public sealed partial class SpatialHistorySystem : EntitySystem
{
    [Dependency] private INetManager _net = default!;
    [Dependency] private IGameTiming _timing = default!;
    [Dependency] private SharedSpatialPhysicsSystem _physics = default!;
    [Dependency] private SharedTransformSystem _transforms = default!;
    [Dependency] private SharedGeometrySystem _geometry = default!;
    private readonly Dictionary<EntityUid, WorldHistory> _worlds = new();
    private BepuSpatialPhysicsBackend? _queryBackend;
    private SpatialHistoricalScene? _queryScene;
    private bool _querying;

    private sealed class WorldHistory(SpatialHistorySettings settings)
    {
        public readonly SpatialHistorySettings Settings = settings;
        public readonly SortedDictionary<GameTick, SpatialHistoricalScene> Frames = new();
        public int Poses;
        public int Shapes;
    }

    public override void Initialize()
    {
        base.Initialize();
        SubscribeLocalEvent<SpatialPhysicsStepCompleted>(OnStep);
        EntityManager.EntityDeleted += OnDeleted;
        EntityManager.ComponentRemoved += OnComponentRemoved;
        EntityManager.AfterEntityFlush += Clear;
    }

    public override void Shutdown()
    {
        EntityManager.EntityDeleted -= OnDeleted;
        EntityManager.ComponentRemoved -= OnComponentRemoved;
        EntityManager.AfterEntityFlush -= Clear;
        Clear();
        base.Shutdown();
    }

    public void Configure(EntityUid world, SpatialHistorySettings settings)
    {
        if (!_net.IsServer || !_physics.TryGetBackend(world, out _))
            throw new InvalidOperationException("History requires an authoritative active spatial physics world.");
        if (_querying) throw new InvalidOperationException("A query policy cannot reconfigure history.");
        settings.Validate();
        Remove(world);
        _worlds.Add(world, new WorldHistory(settings));
    }

    public void Disable(EntityUid world)
    {
        if (!_net.IsServer || _querying) throw new InvalidOperationException("History configuration requires the server outside a query.");
        Remove(world);
    }

    public bool TryGetScene(EntityUid world, GameTick tick, [NotNullWhen(true)] out SpatialHistoricalScene? scene)
    {
        scene = null;
        return _worlds.TryGetValue(world, out var history) && history.Frames.TryGetValue(tick, out scene);
    }

    private void OnDeleted(Entity<MetaDataComponent> entity) => Remove(entity.Owner);
    private void OnComponentRemoved(RemovedComponentEventArgs ev)
    {
        if (ev.BaseArgs.Component is SpatialPhysicsWorldComponent) Remove(ev.BaseArgs.Owner);
    }

    private void OnStep(ref SpatialPhysicsStepCompleted ev)
    {
        var tick = _timing.CurTick;
        foreach (var (world, history) in _worlds.ToArray())
        {
            if (!_physics.TryGetBackend(world, out _)) { Remove(world); continue; }
            // Repeated explicit steps at one engine tick replace that tick, not
            // manufacture additional timestamps or retain obsolete references.
            history.Frames.TryGetValue(tick, out var previousAtTick);
            var previous = history.Frames.Count == 0 ? null : history.Frames.Last().Value;
            var bodies = new List<SpatialHistoryBody>();
            var poses = new Dictionary<EntityUid, RigidTransform3>();
            var geometry = new Dictionary<EntityUid, SpatialGeometrySnapshot>();
            var shapes = 0;
            string? gap = null;
            foreach (var body in _physics.CaptureHistoryBodies(world))
            {
                if ((long) shapes + body.Definition.Parts.Count > history.Settings.MaximumShapeSamples || !AddPose(body.Entity))
                { gap = "The current physical frame exceeds the configured history sample budget."; break; }
                shapes += body.Definition.Parts.Count;
                bodies.Add(body);
                foreach (var part in body.Definition.Parts)
                    if (part.Entity is { } source && !AddPose(source))
                    { gap = "A collision source is unavailable or exceeds the history pose budget."; break; }
                if (gap != null) break;
                foreach (var member in body.AnchoredMembers)
                    if (!AddPose(member.Key)) { gap = "A compound member exceeds the history pose budget."; break; }
                if (gap != null) break;
                foreach (var frame in body.GeometryFrames)
                {
                    if (!AddPose(frame)) { gap = "A geometry frame exceeds the history pose budget."; break; }
                    var component = Comp<SpatialGeometryComponent>(frame);
                    var snapshot = previous != null && previous.Geometries.TryGetValue(frame, out var old)
                        && old.CaptureIdentity == component.CaptureIdentity && old.Revision == component.Revision
                        ? old : _geometry.CaptureSnapshot(frame);
                    geometry.Add(frame, snapshot);
                }
                if (gap != null) break;
            }
            if (!AddPose(world)) gap ??= "The world frame exceeds the history pose budget.";
            if (previousAtTick != null) Evict(history, previousAtTick);
            // Tick age is evaluated even on a failed capture; an overflow cannot
            // keep very old frames alive indefinitely.
            foreach (var stale in history.Frames.Values.Where(f => f.Tick > tick
                         || tick.Value - f.Tick.Value >= history.Settings.RetainedTicks).ToArray()) Evict(history, stale);
            if (gap != null)
            {
                var missing = new SpatialHistoryGap(world, tick, gap);
                RaiseLocalEvent(ref missing);
                continue;
            }
            while (history.Frames.Count != 0 && ((long) history.Poses + poses.Count > history.Settings.MaximumPoseSamples
                || (long) history.Shapes + shapes > history.Settings.MaximumShapeSamples))
                Evict(history, history.Frames.First().Value);
            var scene = new SpatialHistoricalScene(this, world, tick, bodies.ToArray(), poses, geometry, shapes);
            history.Frames.Add(tick, scene);
            history.Poses += poses.Count;
            history.Shapes += shapes;

            bool AddPose(EntityUid uid)
            {
                while (!poses.ContainsKey(uid))
                {
                    if (TerminatingOrDeleted(uid) || !TryComp<TransformComponent>(uid, out var transform)
                        || transform.MapUid != world || poses.Count >= history.Settings.MaximumPoseSamples) return false;
                    poses.Add(uid, _transforms.GetWorldPose(transform));
                    if (!transform.ParentUid.IsValid()) break;
                    uid = transform.ParentUid;
                }
                return true;
            }
        }
    }

    private void Evict(WorldHistory history, SpatialHistoricalScene scene)
    {
        history.Frames.Remove(scene.Tick);
        history.Poses -= scene.Poses.Count;
        history.Shapes -= scene.ShapeCount;
        if (ReferenceEquals(_queryScene, scene)) ClearBackend();
        scene.Expire();
    }

    private void Remove(EntityUid world)
    {
        if (!_worlds.Remove(world, out var history)) return;
        foreach (var scene in history.Frames.Values.ToArray()) Evict(history, scene);
    }

    private void Clear()
    {
        foreach (var world in _worlds.Keys.ToArray()) Remove(world);
        ClearBackend();
    }

    private void ClearBackend()
    {
        _queryBackend?.Dispose();
        _queryBackend = null;
        _queryScene = null;
    }

    internal ISpatialPhysicsBackend BeginQuery(SpatialHistoricalScene scene)
    {
        if (_querying) throw new InvalidOperationException("A history query policy cannot recursively query history.");
        if (!scene.IsAvailable) throw new InvalidOperationException("The requested historical scene has expired.");
        if (!ReferenceEquals(scene, _queryScene))
        {
            ClearBackend();
            var backend = new BepuSpatialPhysicsBackend(Vector3.Zero);
            try
            {
                foreach (var body in scene.Bodies)
                    backend.AddBody(new SpatialBackendBodyDefinition(SpatialBackendMotion.Static, body.Definition.Parts,
                        body.Definition.Mass, body.Definition.Material, body.Definition.CollisionLayer, body.Definition.CollisionMask),
                        new SpatialBackendState(body.State.Pose, Vector3.Zero, Vector3.Zero));
                _queryBackend = backend;
                _queryScene = scene;
            }
            catch { backend.Dispose(); throw; }
        }
        _querying = true;
        return _queryBackend!;
    }

    internal void EndQuery() => _querying = false;
}
