using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using Robust.Client.Timing;
using Robust.Shared.Containers;
using Robust.Shared.GameObjects;
using Robust.Shared.IoC;
using Robust.Shared.Maths;
using Robust.Shared.Network;
using Robust.Shared.Spatial;
using Robust.Shared.Spatial.Physics;
using Robust.Shared.Timing;

namespace Robust.Client.Spatial;

public enum SpatialMotorPredictionStatus : byte
{
    WaitingForState, Active, GeometryUnavailable, OutsideCoverage, PredictionLimit,
    InactiveBody, ConstrainedBody, Disposed
}

/// <summary>
/// Opt-in local capsule prediction. Existing game-state reset, input ACKs and
/// predictive-event replay own reconciliation. Only the selected motor writes
/// predicted ECS motion; remote bodies are derived kinematic collision proxies.
/// </summary>
public sealed partial class SpatialMotorPredictionSystem : EntitySystem, ISpatialMotorPredictionDriver
{
    [Dependency] private IClientGameTiming _timing = default!;
    [Dependency] private SpatialCollisionViewSystem _views = default!;
    [Dependency] private SharedSpatialPhysicsSystem _physics = default!;
    [Dependency] private SharedTransformSystem _transforms = default!;
    [Dependency] private SharedContainerSystem _containers = default!;
    [Dependency] private IClientNetManager _net = default!;
    private readonly Dictionary<EntityUid, SpatialMotorPrediction> _predictions = new();

    public override void Initialize()
    {
        base.Initialize();
        UpdatesOutsidePrediction = true;
        UpdatesAfter.Add(typeof(SharedSpatialPhysicsSystem));
        UpdatesAfter.Add(typeof(SpatialGeometryStreamingSystem));
        _physics.MotorPredictionDriver = this;
        EntityManager.EntityDeleted += OnDeleted;
        EntityManager.AfterEntityFlush += Clear;
        _net.Disconnect += OnDisconnect;
        _transforms.OnGlobalMoveEvent += OnMove;
    }

    public override void Shutdown()
    {
        Clear();
        _physics.MotorPredictionDriver = null;
        EntityManager.EntityDeleted -= OnDeleted;
        EntityManager.AfterEntityFlush -= Clear;
        _net.Disconnect -= OnDisconnect;
        _transforms.OnGlobalMoveEvent -= OnMove;
        base.Shutdown();
    }

    /// <summary>
    /// Content selects its locally controlled motor and certified collision view.
    /// This grants no PVS or server authority. One selected motor per world.
    /// </summary>
    public SpatialMotorPrediction PredictMotor(EntityUid entity, Box3 bounds,
        SpatialCollisionViewLimits limits, int maximumPredictionTicks)
    {
        if (maximumPredictionTicks <= 0 || !TryComp<SpatialBodyComponent>(entity, out var body)
            || !body.HasDefinition || body.Definition.Owner != SpatialMotionOwner.Motor
            || !MetaData(entity).NetEntity.IsValid() || MetaData(entity).NetEntity.IsClientSide()
            || !HasComp<SpatialMotorComponent>(entity) || !TryComp<TransformComponent>(entity, out var transform)
            || transform.MapUid is not { } world || !HasComp<SpatialPhysicsWorldComponent>(world))
            throw new ArgumentException("Prediction requires a known motor in a physical world and a positive tick budget.");
        if (_predictions.Values.Any(p => p.World == world))
            throw new InvalidOperationException("This world already has a selected local motor.");
        var prediction = new SpatialMotorPrediction(this, entity, _views.CreateView(world, bounds, limits), maximumPredictionTicks);
        _predictions.Add(entity, prediction);
        return prediction;
    }

    internal void Release(SpatialMotorPrediction prediction) => _predictions.Remove(prediction.Entity);
    private void Clear() { foreach (var prediction in _predictions.Values.ToArray()) prediction.Dispose(); }
    private void OnDisconnect(object? sender, NetDisconnectedArgs args) => Clear();
    private void OnMove(ref MoveEvent args)
    {
        if (_predictions.TryGetValue(args.Sender, out var prediction) && args.Component.MapUid != prediction.World)
            prediction.Dispose();
    }
    internal void ResetPredictedSolvers()
    {
        foreach (var prediction in _predictions.Values)
        {
            prediction.ResetSolver();
            prediction.Status = SpatialMotorPredictionStatus.WaitingForState;
        }
    }
    private void OnDeleted(Entity<MetaDataComponent> entity)
    {
        foreach (var prediction in _predictions.Values.ToArray())
            if (prediction.Entity == entity.Owner || prediction.World == entity.Owner) prediction.Dispose();
    }

    bool ISpatialMotorPredictionDriver.CanPredict(EntityUid entity) => CanPredict(entity);
    private bool CanPredict(EntityUid entity)
        => _timing.InPrediction && _predictions.TryGetValue(entity, out var prediction) && !prediction.IsDisposed
            && !prediction.View.IsDisposed && Active(entity, prediction.World);

    private bool Active(EntityUid entity, EntityUid world)
        => !TerminatingOrDeleted(entity) && !TerminatingOrDeleted(world)
            && TryComp<SpatialBodyComponent>(entity, out var body) && body.Running && body.HasDefinition
            && body.Definition.Owner == SpatialMotionOwner.Motor && TryComp<SpatialMotorComponent>(entity, out var motor) && motor.Running
            && Transform(entity).MapUid == world && !MetaData(entity).EntityPaused && !MetaData(world).EntityPaused
            && (MetaData(entity).Flags & MetaDataFlags.Detached) == 0 && !_containers.IsEntityOrParentInContainer(entity);

    public override void Update(float frameTime)
    {
        foreach (var prediction in _predictions.Values.ToArray())
        {
            if (prediction.View.IsDisposed || TerminatingOrDeleted(prediction.Entity)
                || Transform(prediction.Entity).MapUid != prediction.World)
            { prediction.Dispose(); continue; }
            if (!_timing.InPrediction || frameTime <= 0) continue;
            if (!Prepare(prediction)) continue;
            var solver = prediction.Solver!;
            var body = Comp<SpatialBodyComponent>(prediction.Entity);
            var motor = Comp<SpatialMotorComponent>(prediction.Entity);
            SyncMotor(solver, prediction.Entity, body);
            using var coverage = solver.Backend.BeginMotorCoverage(solver.Motor, prediction.View.Bounds);
            solver.Backend.PrepareMotorStep(solver.Motor, motor.Settings, motor.Intent, motor.Support != null, frameTime);
            solver.Backend.Step(frameTime);
            var state = solver.Backend.GetState(solver.Motor);
            SpatialMotorSupport? support = null;
            if (motor.Intent.Mode == SpatialMotorMode.Walking
                && solver.Backend.FindMotorSupport(solver.Motor, motor.Settings) is { } ground
                && ground.Distance <= motor.Settings.ContactTolerance * 2
                && Vector3.Dot(state.LinearVelocity - ground.Velocity, ground.Normal) <= 0.25f)
                support = new SpatialMotorSupport(new SpatialQueryCandidate(ground.Source.Entity, ground.Source.Surface,
                    ground.Source.MaterialProfile, ground.Source.Revision), ground.Point, ground.Normal, ground.Velocity, ground.Distance);
            if (!coverage.Available || !_views.IsCurrent(prediction.View, solver.Scene.State, solver.Scene.GeometrySequence))
            {
                prediction.Block(coverage.Available ? SpatialMotorPredictionStatus.GeometryUnavailable : SpatialMotorPredictionStatus.OutsideCoverage);
                continue;
            }
            // Dirty components participate in ClientGameStateManager's normal
            // authoritative reset before replay. No contact/landing/game events
            // are raised here: their authoritative consequences run on server.
            _transforms.CommitSpatialPoses(new Dictionary<EntityUid, RigidTransform3> { [prediction.Entity] = state.Pose }, () =>
            {
                body._linearVelocity = state.LinearVelocity;
                body._angularVelocity = state.AngularVelocity;
                motor._support = support;
                Dirty(prediction.Entity, body);
                Dirty(prediction.Entity, motor);
            });
            solver.Tick = _timing.CurTick;
            prediction.LastPredictedTick = solver.Tick;
            prediction.PredictedSteps++;
            prediction.Status = SpatialMotorPredictionStatus.Active;
            solver.Backend.DrainContactEvents();
        }
    }

    private bool Prepare(SpatialMotorPrediction prediction, bool forInput = false)
    {
        var capture = prediction.View.Current;
        if (!ReferenceEquals(prediction.Capture, capture))
        {
            prediction.ResetSolver();
            prediction.Capture = capture;
            prediction.Status = SpatialMotorPredictionStatus.WaitingForState;
        }
        if (capture == null || prediction.Blocked) return false;
        if (!Active(prediction.Entity, prediction.World))
        { prediction.Block(SpatialMotorPredictionStatus.InactiveBody); return false; }
        if (_timing.CurTick <= capture.Tick || _timing.CurTick.Value - capture.Tick.Value > prediction.MaximumPredictionTicks)
        { prediction.Block(SpatialMotorPredictionStatus.PredictionLimit); return false; }
        if (prediction.Solver is { } current)
        {
            if (current.Tick + 1 != _timing.CurTick && !(forInput && current.Tick == _timing.CurTick))
            { prediction.Block(SpatialMotorPredictionStatus.WaitingForState); return false; }
            if (!_views.IsCurrent(prediction.View, capture, current.Scene.GeometrySequence))
            { prediction.Block(SpatialMotorPredictionStatus.GeometryUnavailable); return false; }
            return true;
        }
        // If geometry arrives mid-replay, do not skip the earlier inputs. Resume
        // from the next complete authoritative state and its normal full replay.
        if (capture.Tick + 1 != _timing.CurTick)
        { prediction.Block(SpatialMotorPredictionStatus.WaitingForState); return false; }
        var scene = _views.CaptureScene(prediction.View);
        if (scene == null || capture.Physics == null)
        { prediction.Block(SpatialMotorPredictionStatus.GeometryUnavailable); return false; }
        if (!scene.Bodies.Any(b => b.Entity == prediction.Entity)
            || capture.Entities[prediction.World].Paused || capture.Entities[prediction.Entity].Paused)
        { prediction.Block(SpatialMotorPredictionStatus.InactiveBody); return false; }
        if (capture.Entities.Values.Any(e => !e.Paused && e.Constraint is { Kind: SpatialConstraintKind.Fixed } c
                && (c.BodyA == prediction.Entity || c.BodyB == prediction.Entity)))
        { prediction.Block(SpatialMotorPredictionStatus.ConstrainedBody); return false; }
        if (!PredictionSolver.TryCreate(scene, prediction.Entity, out var solver))
        { prediction.Block(SpatialMotorPredictionStatus.GeometryUnavailable); return false; }
        prediction.Solver = solver;
        return true;
    }

    private void SyncMotor(PredictionSolver solver, EntityUid uid, SpatialBodyComponent body)
    {
        var state = new SpatialBackendState(_transforms.GetWorldPose(uid), body.LinearVelocity, body.AngularVelocity);
        if (solver.DefinitionRevision != body.DefinitionRevision)
        {
            solver.Backend.RemoveBody(solver.Motor);
            solver.Motor = solver.Backend.AddBody(MotorDefinition(uid, body.Definition, body.DefinitionRevision), state);
            solver.DefinitionRevision = body.DefinitionRevision;
            solver.BodyIds[uid] = solver.Motor;
            solver.BindConstraints();
        }
        else solver.Backend.SetState(solver.Motor, state);
    }

    bool ISpatialMotorPredictionDriver.TryResize(EntityUid uid, float radius, float length, ulong revision)
    {
        if (!CanPredict(uid)) throw new InvalidOperationException("Motor resize requires active prediction.");
        var prediction = _predictions[uid];
        var body = Comp<SpatialBodyComponent>(uid);
        if (body.DefinitionRevision != revision) throw new InvalidOperationException("Stale motor definition revision.");
        var shape = new SpatialCapsuleShape(radius, length);
        if (!Prepare(prediction, forInput: true)) return false;
        var solver = prediction.Solver!;
        SyncMotor(solver, uid, body);
        var definition = body.Definition;
        var current = definition.Colliders[0];
        var capsule = (SpatialCapsuleShape) current.Shape;
        var offset = current.Offset + Vector3.UnitZ * (radius + length * 0.5f - capsule.Radius - capsule.Length * 0.5f);
        var collider = new SpatialColliderDefinition(shape, new RigidTransform3(offset, current.Rotation), current.MassWeight);
        using var coverage = solver.Backend.BeginMotorCoverage(solver.Motor, prediction.View.Bounds);
        if (!solver.Backend.IsMotorResizeClear(solver.Motor, new SpatialBackendPart(shape.ToBackend(), collider.LocalPose)) || !coverage.Available)
            return false;
        body._definition = new SpatialBodyDefinition(SpatialMotionOwner.Motor, new[] { collider },
            mass: definition.Mass, material: definition.Material, collisionLayer: definition.CollisionLayer, collisionMask: definition.CollisionMask);
        body._definitionRevision = checked(revision + 1);
        Dirty(uid, body);
        return true;
    }

    private static SpatialBackendBodyDefinition MotorDefinition(EntityUid uid, SpatialBodyDefinition body, ulong revision)
        => new(SpatialBackendMotion.Dynamic, body.Colliders.Select(c => new SpatialBackendPart(c.Shape.ToBackend(), c.LocalPose,
            uid, Revision: revision, MassWeight: c.MassWeight)), body.Mass, body.Material.ToBackend(), body.CollisionLayer,
            body.CollisionMask, lockRotation: true);

    internal sealed class PredictionSolver : IDisposable
    {
        public readonly SpatialCollisionViewSystem.CapturedScene Scene;
        public readonly BepuSpatialPhysicsBackend Backend;
        public readonly Dictionary<EntityUid, SpatialBackendBodyId> BodyIds = new();
        private readonly Dictionary<EntityUid, EntityUid> _roots;
        private readonly List<SpatialConstraintDefinition> _constraints;
        private readonly List<SpatialBackendConstraintId> _constraintIds = new();
        public SpatialBackendBodyId Motor;
        public ulong DefinitionRevision;
        public GameTick Tick;

        public static bool TryCreate(SpatialCollisionViewSystem.CapturedScene scene, EntityUid motor, out PredictionSolver? solver)
        {
            solver = null;
            var records = scene.Bodies.ToDictionary(b => b.Entity);
            var roots = new Dictionary<EntityUid, EntityUid>();
            foreach (var collider in scene.Bodies)
            {
                var entity = scene.State.Entities[collider.Entity];
                if (entity.Body?.Owner != SpatialMotionOwner.AnchoredCompoundChild) continue;
                var root = entity.Parent;
                while (true)
                {
                    if (!scene.State.Entities.TryGetValue(root, out var parent)) return false;
                    if (parent.Body?.Owner != SpatialMotionOwner.AnchoredCompoundChild
                        && (parent.Body != null || records.ContainsKey(root)))
                    {
                        if (!records.ContainsKey(root)) return false;
                        roots.Add(collider.Entity, root);
                        break;
                    }
                    root = parent.Parent;
                }
            }
            var constraints = scene.State.Entities.Values.Where(e => !e.Paused && e.Constraint is { } c
                && (c.BodyA == motor || c.BodyB == motor)).Select(e => e.Constraint!).ToList();
            foreach (var constraint in constraints)
                if (!records.ContainsKey(roots.GetValueOrDefault(constraint.BodyA, constraint.BodyA))
                    || !records.ContainsKey(roots.GetValueOrDefault(constraint.BodyB, constraint.BodyB))) return false;
            solver = new PredictionSolver(scene, motor, records, roots, constraints);
            return true;
        }

        private PredictionSolver(SpatialCollisionViewSystem.CapturedScene scene, EntityUid motor,
            Dictionary<EntityUid, SpatialCollisionViewSystem.CapturedCollider> records, Dictionary<EntityUid, EntityUid> roots,
            List<SpatialConstraintDefinition> constraints)
        {
            Scene = scene; Tick = scene.State.Tick;
            _roots = roots; _constraints = constraints;
            var settings = scene.State.Physics!.Value;
            Backend = new BepuSpatialPhysicsBackend(settings.Gravity, settings.Substeps, settings.SolverIterations);
            try
            {
                var children = new Dictionary<EntityUid, List<SpatialCompoundMember>>();
                foreach (var (uid, root) in roots)
                {
                    var collider = records[uid];
                    if (!children.TryGetValue(root, out var group)) children.Add(root, group = new());
                    group.Add(new SpatialCompoundMember(collider.Entity, RelativePose(collider.Entity, root), collider.Definition));
                }
                foreach (var collider in scene.Bodies)
                {
                    var entity = scene.State.Entities[collider.Entity];
                    if (entity.Body?.Owner == SpatialMotionOwner.AnchoredCompoundChild) continue;
                    var source = collider.Definition;
                    var own = collider.Entity == motor;
                    var moving = !entity.Paused && entity.Body?.Owner is SpatialMotionOwner.DynamicBody or SpatialMotionOwner.Motor;
                    var definition = own ? MotorDefinition(motor, entity.Body!, entity.DefinitionRevision)
                        : new SpatialBackendBodyDefinition(moving ? SpatialBackendMotion.Kinematic : SpatialBackendMotion.Static,
                            source.Parts, source.Mass, source.Material, source.CollisionLayer, source.CollisionMask);
                    if (children.TryGetValue(collider.Entity, out var group)) definition = SpatialCompoundLayout.Compose(collider.Entity, definition, group);
                    var id = Backend.AddBody(definition, new SpatialBackendState(collider.Pose,
                        moving || own ? entity.LinearVelocity : Vector3.Zero, moving || own ? entity.AngularVelocity : Vector3.Zero));
                    BodyIds.Add(collider.Entity, id);
                    if (own) { Motor = id; DefinitionRevision = entity.DefinitionRevision; }
                }
                BindConstraints();
            }
            catch { Backend.Dispose(); throw; }
        }

        public void BindConstraints()
        {
            foreach (var id in _constraintIds)
                if (Backend.HasConstraint(id)) Backend.RemoveConstraint(id);
            _constraintIds.Clear();
            foreach (var definition in _constraints)
            {
                var rootA = _roots.GetValueOrDefault(definition.BodyA, definition.BodyA);
                var rootB = _roots.GetValueOrDefault(definition.BodyB, definition.BodyB);
                var relativeA = RelativePose(definition.BodyA, rootA);
                var relativeB = RelativePose(definition.BodyB, rootB);
                var resolved = definition.ToBackend(BodyIds[rootA], BodyIds[rootB]) with
                {
                    LocalAnchorA = relativeA.TransformPoint(definition.LocalAnchorA),
                    LocalAnchorB = relativeB.TransformPoint(definition.LocalAnchorB),
                    RelativePose = relativeA.Compose(definition.RelativePose).Compose(relativeB.Inverse())
                };
                _constraintIds.Add(Backend.AddConstraint(resolved));
            }
        }

        private RigidTransform3 RelativePose(EntityUid entity, EntityUid root)
        {
            var pose = RigidTransform3.Identity;
            while (entity != root)
            {
                var frame = Scene.State.Entities[entity];
                pose = frame.LocalPose.Compose(pose);
                entity = frame.Parent;
            }
            return pose;
        }

        public void Dispose() => Backend.Dispose();
    }
}

/// <summary>Content-owned selection lifetime and budgets; no independent entity or input queue.</summary>
public sealed class SpatialMotorPrediction : IDisposable
{
    private readonly SpatialMotorPredictionSystem _owner;
    private readonly int _thread = Environment.CurrentManagedThreadId;
    internal readonly SpatialCollisionView View;
    internal SpatialCollisionViewSystem.Capture? Capture;
    internal SpatialMotorPredictionSystem.PredictionSolver? Solver;
    internal bool Blocked;
    public EntityUid Entity { get; }
    public EntityUid World => View.World;
    public Box3 Bounds => View.Bounds;
    public int MaximumPredictionTicks { get; }
    public SpatialMotorPredictionStatus Status { get; internal set; } = SpatialMotorPredictionStatus.WaitingForState;
    public GameTick? LastPredictedTick { get; internal set; }
    public ulong PredictedSteps { get; internal set; }
    public bool IsDisposed { get; private set; }

    internal SpatialMotorPrediction(SpatialMotorPredictionSystem owner, EntityUid entity, SpatialCollisionView view, int maximumTicks)
    { _owner = owner; Entity = entity; View = view; MaximumPredictionTicks = maximumTicks; }
    public void SetBounds(Box3 bounds)
    {
        CheckThread(); ObjectDisposedException.ThrowIf(IsDisposed, this);
        if (bounds == View.Bounds) return;
        View.SetBounds(bounds); ResetSolver(); Capture = null; Status = SpatialMotorPredictionStatus.WaitingForState;
    }
    internal void Block(SpatialMotorPredictionStatus status)
    { ResetSolver(); Blocked = true; Status = status; }
    internal void ResetSolver()
    { Solver?.Dispose(); Solver = null; Blocked = false; }
    public void Dispose()
    {
        CheckThread();
        if (IsDisposed) return;
        ResetSolver(); Capture = null; View.Dispose(); IsDisposed = true;
        Status = SpatialMotorPredictionStatus.Disposed; _owner.Release(this);
    }
    private void CheckThread()
    {
        if (_thread != Environment.CurrentManagedThreadId) throw new InvalidOperationException("Motor prediction requires its client thread.");
    }
}
