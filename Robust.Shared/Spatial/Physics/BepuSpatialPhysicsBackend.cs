using System;
using System.Collections.Generic;
using System.Numerics;
using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.Constraints;
using BepuPhysics.Trees;
using BepuUtilities;
using BepuUtilities.Memory;
using Robust.Shared.Maths;

namespace Robust.Shared.Spatial.Physics;

/// <summary>
/// One trusted BEPU simulation per spatial world. BEPU positions are derived
/// solver state; ECS must commit results to the existing canonical Transform.
/// Body handles, compound shape indices and pooled buffers remain private.
/// </summary>
internal sealed partial class BepuSpatialPhysicsBackend : ISpatialPhysicsBackend
{
    private readonly BufferPool _pool = new();
    private readonly Simulation _simulation;
    private readonly Vector3 _gravity;
    private readonly Dictionary<SpatialBackendBodyId, Entry> _entries = new();
    private readonly Dictionary<int, Entry> _statics = new();
    private readonly Dictionary<int, Entry> _bodies = new();
    private ulong _nextId;
    private bool _disposed;

    private sealed class Entry
    {
        public required SpatialBackendBodyId Id;
        public required SpatialBackendBodyDefinition Definition;
        public required TypedIndex Shape;
        public Vector3 Center;
        public float Radius;
        public StaticHandle Static;
        public BodyHandle Body;
    }

    public int BodyCount => _entries.Count;

    public BepuSpatialPhysicsBackend(Vector3 gravity, int substeps = 1, int solverIterations = 8)
    {
        if (!SpatialBackendBodyDefinition.Finite(gravity))
            throw new ArgumentOutOfRangeException(nameof(gravity));
        if (substeps <= 0 || solverIterations <= 0)
            throw new ArgumentOutOfRangeException(nameof(substeps));
        _gravity = gravity;
        var timestepper = new DefaultTimestepper();
        timestepper.BeforeCollisionDetection += ExpandRotatingCompoundBounds;
        _simulation = Simulation.Create(_pool, new Contacts(this), new Integrator(gravity),
            new SolveDescription(solverIterations, substeps), timestepper);
        SpatialSweepRegistration.Register(_simulation.NarrowPhase.SweepTaskRegistry);
    }

    public SpatialBackendBodyId AddBody(SpatialBackendBodyDefinition definition, SpatialBackendState state)
    {
        ThrowIfDisposed();
        ArgumentNullException.ThrowIfNull(definition);
        ValidateState(state, definition.Motion);
        var id = new SpatialBackendBodyId(checked(++_nextId));
        var builder = new CompoundBuilder(_pool, _simulation.Shapes, definition.Parts.Count);
        var allocated = new List<TypedIndex>();
        TypedIndex compound = default;
        Buffer<CompoundChild> children = default;
        try
        {
            double totalWeight = 0;
            foreach (var part in definition.Parts)
                totalWeight += part.MassWeight;
            foreach (var part in definition.Parts)
            {
                var mass = (float) (definition.Mass * (part.MassWeight / totalWeight));
                if (mass <= 0 || !float.IsFinite(1 / mass))
                    throw new ArgumentException("Compound part mass is outside the representable range.");
                var local = part.LocalPose;
                switch (part.Shape)
                {
                    case SpatialBox box:
                        AddConvex(new Box(box.Size.X, box.Size.Y, box.Size.Z), local, mass);
                        break;
                    case SpatialSphere sphere:
                        AddConvex(new Sphere(sphere.Radius), local, mass);
                        break;
                    case SpatialCapsule capsule:
                        // BEPU capsules point along Y; the engine contract uses Z-up.
                        local = local.Compose(new RigidTransform3(Vector3.Zero,
                            Quaternion.CreateFromAxisAngle(Vector3.UnitX, MathF.PI / 2)));
                        AddConvex(new Capsule(capsule.Radius, capsule.Length), local, mass);
                        break;
                }
            }

            Vector3 center = default;
            BodyInertia inertia = default;
            if (definition.Motion == SpatialBackendMotion.Dynamic)
            {
                builder.BuildDynamicCompound(out children, out inertia, out center);
                ValidateInertia(inertia);
            }
            else
                builder.BuildKinematicCompound(out children);
            // BigCompound provides its own child acceleration structure. Parts keep
            // their original indices and source identities through ray traversal.
            compound = _simulation.Shapes.Add(new BigCompound(children, _simulation.Shapes, _pool));
            children = default; // The compound owns this buffer now.
            var entry = new Entry { Id = id, Definition = definition, Shape = compound, Center = center };
            _simulation.Shapes.UpdateBounds(new RigidPose(Vector3.Zero), ref compound, out var localBounds);
            entry.Radius = Vector3.Max(Vector3.Abs(localBounds.Min), Vector3.Abs(localBounds.Max)).Length();
            if (!float.IsFinite(entry.Radius))
                throw new ArgumentException("Compound radius exceeds the finite solver range.");
            var pose = ToSolverPose(state.Pose, center);
            var velocity = ToSolverVelocity(state, center);
            ValidateSolverState(pose, velocity);
            if (definition.Motion == SpatialBackendMotion.Static)
            {
                entry.Static = _simulation.Statics.Add(new StaticDescription(pose, compound));
                _statics.Add(entry.Static.Value, entry);
            }
            else
            {
                var collidable = new CollidableDescription(compound, 1f, ContinuousDetection.Continuous(1e-6f, 1e-6f));
                var description = definition.Motion == SpatialBackendMotion.Dynamic
                    ? BodyDescription.CreateDynamic(pose, velocity, inertia, collidable, 0.01f)
                    : BodyDescription.CreateKinematic(pose, velocity, collidable, 0.01f);
                entry.Body = _simulation.Bodies.Add(description);
                _bodies.Add(entry.Body.Value, entry);
            }
            _entries.Add(id, entry);
            return id;
        }
        catch
        {
            if (compound.Exists)
                _simulation.Shapes.RecursivelyRemoveAndDispose(compound, _pool);
            else
            {
                if (children.Allocated)
                    _pool.Return(ref children);
                foreach (var shape in allocated)
                    _simulation.Shapes.RemoveAndDispose(shape, _pool);
            }
            throw;
        }
        finally { builder.Dispose(); }

        void AddConvex<T>(T shape, RigidTransform3 local, float mass) where T : unmanaged, IConvexShape
        {
            shape.ComputeBounds(local.Rotation, out var min, out var max);
            _ = new Box3(min + local.Position, max + local.Position);
            var inertia = definition.Motion == SpatialBackendMotion.Dynamic ? shape.ComputeInertia(mass) : default;
            if (definition.Motion == SpatialBackendMotion.Dynamic)
                ValidateInertia(inertia);
            var shapeIndex = _simulation.Shapes.Add(shape);
            allocated.Add(shapeIndex);
            var pose = new RigidPose(local.Position, local.Rotation);
            if (definition.Motion == SpatialBackendMotion.Dynamic)
                builder.Add(shapeIndex, pose, inertia.InverseInertiaTensor, mass);
            else
                builder.AddForKinematic(shapeIndex, pose, mass);
        }
    }

    public void RemoveBody(SpatialBackendBodyId id)
    {
        ThrowIfDisposed();
        var entry = _entries[id];
        RemoveBodyConstraints(id);
        if (entry.Definition.Motion == SpatialBackendMotion.Static)
        {
            _simulation.Statics.Remove(entry.Static);
            _statics.Remove(entry.Static.Value);
        }
        else
        {
            _simulation.Bodies.Remove(entry.Body);
            _bodies.Remove(entry.Body.Value);
        }
        _simulation.Shapes.RecursivelyRemoveAndDispose(entry.Shape, _pool);
        _entries.Remove(id);
    }

    public SpatialBackendState GetState(SpatialBackendBodyId id)
    {
        ThrowIfDisposed();
        var entry = _entries[id];
        if (entry.Definition.Motion == SpatialBackendMotion.Static)
        {
            var pose = _simulation.Statics.GetStaticReference(entry.Static).Pose;
            return new SpatialBackendState(new RigidTransform3(pose.Position, pose.Orientation), Vector3.Zero, Vector3.Zero);
        }
        var body = _simulation.Bodies.GetBodyReference(entry.Body);
        var offset = Vector3.Transform(entry.Center, body.Pose.Orientation);
        return new SpatialBackendState(new RigidTransform3(body.Pose.Position - offset, body.Pose.Orientation),
            body.Velocity.Linear - Vector3.Cross(body.Velocity.Angular, offset), body.Velocity.Angular);
    }

    public void SetState(SpatialBackendBodyId id, SpatialBackendState state)
    {
        ThrowIfDisposed();
        var entry = _entries[id];
        ValidateState(state, entry.Definition.Motion);
        var pose = ToSolverPose(state.Pose, entry.Center);
        var velocity = ToSolverVelocity(state, entry.Center);
        ValidateSolverState(pose, velocity);
        if (entry.Definition.Motion == SpatialBackendMotion.Static)
            _simulation.Statics.ApplyDescription(entry.Static, new StaticDescription(pose, entry.Shape));
        else
        {
            var body = _simulation.Bodies.GetBodyReference(entry.Body);
            body.Awake = true;
            body.Pose = pose;
            body.Velocity = velocity;
            body.UpdateBounds();
        }
        UpdateConstraintAnchors(entry);
    }

    public void Step(float fixedDelta)
    {
        ThrowIfDisposed();
        if (!float.IsFinite(fixedDelta) || fixedDelta <= 0)
            throw new ArgumentOutOfRangeException(nameof(fixedDelta));
        _simulation.Timestep(fixedDelta);
    }

    private unsafe void ExpandRotatingCompoundBounds(float dt, IThreadDispatcher? dispatcher)
    {
        // BEPU 2.4's compound batcher clamps each child's tangential *velocity*
        // to its offset radius. That bound misses fast rotation of offset parts.
        // Refit active leaves before collision detection with a conservative
        // displacement bound for every point relative to the compound origin.
        foreach (var entry in _bodies.Values)
        {
            var body = _simulation.Bodies.GetBodyReference(entry.Body);
            if (!body.Awake || body.Velocity.Angular == Vector3.Zero)
                continue;
            _simulation.Shapes.UpdateBounds(body.Pose, ref entry.Shape, out var bounds);
            var angular = Math.Min((double) body.Velocity.Angular.Length() * dt * entry.Radius, 2d * entry.Radius);
            var expansion = new Vector3((float) angular);
            var translation = body.Velocity.Linear * dt;
            var gravity = entry.Definition.Motion == SpatialBackendMotion.Dynamic ? _gravity * (dt * dt) : Vector3.Zero;
            var min = bounds.Min - expansion + Vector3.Min(Vector3.Zero, translation) + Vector3.Min(Vector3.Zero, gravity);
            var max = bounds.Max + expansion + Vector3.Max(Vector3.Zero, translation) + Vector3.Max(Vector3.Zero, gravity);
            _simulation.BroadPhase.GetActiveBoundsPointers(body.Collidable.BroadPhaseIndex, out var currentMin, out var currentMax);
            *currentMin = Vector3.Min(*currentMin, min);
            *currentMax = Vector3.Max(*currentMax, max);
        }
    }

    public SpatialBackendHit? RayCast(Vector3 origin, Vector3 unitDirection, float distance, uint mask,
        SpatialBackendBodyId? ignoredBody = null, Func<SpatialBackendPart, bool>? filter = null)
    {
        ThrowIfDisposed();
        if (!SpatialBackendBodyDefinition.Finite(origin) || !SpatialBackendBodyDefinition.Finite(unitDirection)
            || MathF.Abs(unitDirection.LengthSquared() - 1) > 1e-4f || !float.IsFinite(distance) || distance <= 0)
            throw new ArgumentException("A spatial ray requires finite origin, unit direction and positive distance.");
        var handler = new RayHandler(this, mask, ignoredBody, filter);
        _simulation.RayCast(origin, unitDirection, distance, ref handler);
        return handler.Hit;
    }

    public void Dispose()
    {
        if (_disposed)
            return;
        _disposed = true;
        try { _simulation.Dispose(); }
        finally
        {
            _pool.Clear();
            _entries.Clear();
            _bodies.Clear();
            _statics.Clear();
            _constraints.Clear();
            _bodyConstraints.Clear();
            _disabledContactPairs.Clear();
        }
    }

    private void ThrowIfDisposed() => ObjectDisposedException.ThrowIf(_disposed, this);

    private Entry GetEntry(CollidableReference reference)
        => reference.Mobility == CollidableMobility.Static
            ? _statics[reference.StaticHandle.Value] : _bodies[reference.BodyHandle.Value];

    private static void ValidateState(SpatialBackendState state, SpatialBackendMotion motion)
    {
        if (!state.Pose.IsValid || !SpatialBackendBodyDefinition.Finite(state.LinearVelocity)
            || !SpatialBackendBodyDefinition.Finite(state.AngularVelocity))
            throw new ArgumentException("Body pose and velocities must be finite.");
        if (motion == SpatialBackendMotion.Static && (state.LinearVelocity != Vector3.Zero || state.AngularVelocity != Vector3.Zero))
            throw new ArgumentException("A static body cannot have a velocity.");
    }

    private static RigidPose ToSolverPose(RigidTransform3 pose, Vector3 center)
        => new(pose.TransformPoint(center), pose.Rotation);

    private static BodyVelocity ToSolverVelocity(SpatialBackendState state, Vector3 center)
        => new(state.LinearVelocity + Vector3.Cross(state.AngularVelocity, state.Pose.TransformVector(center)), state.AngularVelocity);

    private static void ValidateSolverState(RigidPose pose, BodyVelocity velocity)
    {
        if (!SpatialBackendBodyDefinition.Finite(pose.Position) || !SpatialBackendBodyDefinition.Finite(velocity.Linear))
            throw new ArgumentException("Center-of-mass conversion exceeds the finite solver range.");
    }

    private static void ValidateInertia(BodyInertia inertia)
    {
        var tensor = inertia.InverseInertiaTensor;
        if (!float.IsFinite(inertia.InverseMass) || inertia.InverseMass <= 0
            || !float.IsFinite(tensor.XX) || tensor.XX <= 0 || !float.IsFinite(tensor.YY) || tensor.YY <= 0
            || !float.IsFinite(tensor.ZZ) || tensor.ZZ <= 0 || !float.IsFinite(tensor.YX)
            || !float.IsFinite(tensor.ZX) || !float.IsFinite(tensor.ZY))
            throw new ArgumentException("Shape mass or inertia exceeds the finite solver range.");
    }

    private struct RayHandler(BepuSpatialPhysicsBackend owner, uint mask, SpatialBackendBodyId? ignored,
        Func<SpatialBackendPart, bool>? filter) : IRayHitHandler
    {
        public SpatialBackendHit? Hit;
        public bool AllowTest(CollidableReference collidable)
        {
            var entry = owner.GetEntry(collidable);
            return entry.Id != ignored && (entry.Definition.CollisionLayer & mask) != 0;
        }
        public bool AllowTest(CollidableReference collidable, int childIndex)
            => filter?.Invoke(owner.GetEntry(collidable).Definition.Parts[childIndex]) ?? true;
        public void OnRayHit(in RayData ray, ref float maximumT, float t, in Vector3 normal,
            CollidableReference collidable, int childIndex)
        {
            var entry = owner.GetEntry(collidable);
            maximumT = t;
            Hit = new SpatialBackendHit(entry.Id, entry.Definition.Parts[childIndex], ray.Origin + ray.Direction * t, normal, t);
        }
    }

    private struct Integrator(Vector3 gravity) : IPoseIntegratorCallbacks
    {
        private Vector3Wide _step;
        public AngularIntegrationMode AngularIntegrationMode => AngularIntegrationMode.Nonconserving;
        public bool AllowSubstepsForUnconstrainedBodies => false;
        public bool IntegrateVelocityForKinematics => false;
        public void Initialize(Simulation simulation) { }
        public void PrepareForIntegration(float dt) => _step = Vector3Wide.Broadcast(gravity * dt);
        public void IntegrateVelocity(Vector<int> indices, Vector3Wide position, QuaternionWide orientation,
            BodyInertiaWide inertia, Vector<int> mask, int worker, Vector<float> dt, ref BodyVelocityWide velocity)
            => velocity.Linear += _step;
    }

    private struct Contacts(BepuSpatialPhysicsBackend owner) : INarrowPhaseCallbacks
    {
        public void Initialize(Simulation simulation) { }
        public void Dispose() { }
        public bool AllowContactGeneration(int worker, CollidableReference a, CollidableReference b, ref float margin)
        {
            if (a.Mobility != CollidableMobility.Dynamic && b.Mobility != CollidableMobility.Dynamic)
                return false;
            if (owner._disabledContactPairs.ContainsKey(ContactPair(owner.GetEntry(a).Id, owner.GetEntry(b).Id)))
                return false;
            var first = owner.GetEntry(a).Definition;
            var second = owner.GetEntry(b).Definition;
            return (first.CollisionLayer & second.CollisionMask) != 0 && (second.CollisionLayer & first.CollisionMask) != 0;
        }
        public bool AllowContactGeneration(int worker, CollidablePair pair, int a, int b) => true;
        public bool ConfigureContactManifold<T>(int worker, CollidablePair pair, ref T manifold,
            out PairMaterialProperties material) where T : unmanaged, IContactManifold<T>
        {
            var a = owner.GetEntry(pair.A).Definition.Material;
            var b = owner.GetEntry(pair.B).Definition.Material;
            material = new PairMaterialProperties
            {
                FrictionCoefficient = (float) Math.Sqrt((double) a.Friction * b.Friction),
                MaximumRecoveryVelocity = Math.Min(a.RecoveryVelocity, b.RecoveryVelocity),
                SpringSettings = new SpringSettings(Math.Min(a.SpringFrequency, b.SpringFrequency), Math.Max(a.Damping, b.Damping))
            };
            return true;
        }
        public bool ConfigureContactManifold(int worker, CollidablePair pair, int a, int b,
            ref ConvexContactManifold manifold) => true;
    }
}
