using System;
using System.Collections.Generic;
using System.Numerics;
using Robust.Shared.GameObjects;
using Robust.Shared.GameStates;
using Robust.Shared.Maths;

namespace Robust.Shared.Spatial.Physics;

public sealed partial class SharedSpatialPhysicsSystem
{
    private readonly Dictionary<EntityUid, SpatialMotorComponent> _motors = new();
    private readonly Dictionary<EntityUid, Vector3> _motorIncomingVelocity = new();

    private void InitializeMotors()
    {
        SubscribeLocalEvent<SpatialMotorComponent, ComponentStartup>(OnMotorStartup);
        SubscribeLocalEvent<SpatialMotorComponent, ComponentShutdown>(OnMotorShutdown);
        SubscribeLocalEvent<SpatialMotorComponent, ComponentGetState>(OnMotorGetState);
        SubscribeLocalEvent<SpatialMotorComponent, ComponentHandleState>(OnMotorHandleState);
    }

    public void ConfigureInitialMotor(Entity<SpatialMotorComponent> motor, SpatialMotorSettings settings,
        SpatialMotorIntent intent = default)
    {
        RequireServer();
        if (MetaData(motor).EntityLifeStage != EntityLifeStage.PreInit)
            throw new InvalidOperationException("Initial motor configuration requires an uninitialized entity.");
        settings.Validate();
        intent.Validate();
        motor.Comp._settings = settings;
        motor.Comp._intent = intent;
    }

    public void SetMotorIntent(Entity<SpatialMotorComponent> motor, SpatialMotorIntent intent)
    {
        RequireMotorAuthority(motor);
        if (TerminatingOrDeleted(motor) || !motor.Comp.Running)
            throw new InvalidOperationException("Motor intent requires a running component.");
        intent.Validate();
        motor.Comp._intent = intent;
        Dirty(motor);
    }

    public void SetMotorSettings(Entity<SpatialMotorComponent> motor, SpatialMotorSettings settings)
    {
        RequireMotorAuthority(motor);
        if (TerminatingOrDeleted(motor) || !motor.Comp.Running)
            throw new InvalidOperationException("Motor settings require a running component.");
        settings.Validate();
        motor.Comp._settings = settings;
        Dirty(motor);
    }

    /// <summary>Change capsule dimensions while preserving its foot in the entity frame.</summary>
    public bool TryResizeMotor(Entity<SpatialMotorComponent> motor, float radius, float length, ulong expectedDefinitionRevision)
    {
        RequireMotorAuthority(motor);
        if (!_net.IsServer)
            return MotorPredictionDriver!.TryResize(motor, radius, length, expectedDefinitionRevision);
        if (TerminatingOrDeleted(motor) || !motor.Comp.Running || !TryComp<SpatialBodyComponent>(motor, out var body)
            || body.Definition.Owner != SpatialMotionOwner.Motor || body.DefinitionRevision != expectedDefinitionRevision
            || !_bindings.TryGetValue(motor, out var binding) || binding.Paused || MetaData(binding.World).EntityPaused)
            throw new InvalidOperationException("Motor resize requires an active body and current definition revision.");
        var shape = new SpatialCapsuleShape(radius, length);
        var current = body.Definition;
        var collider = current.Colliders[0];
        var capsule = (SpatialCapsuleShape) collider.Shape;
        var offset = collider.Offset + Vector3.UnitZ * (radius + length * 0.5f - capsule.Radius - capsule.Length * 0.5f);
        var replacement = new SpatialColliderDefinition(shape, new RigidTransform3(offset, collider.Rotation), collider.MassWeight);
        var backend = _worlds[binding.World];
        if (!backend.IsMotorResizeClear(binding.Body, new SpatialBackendPart(shape.ToBackend(), replacement.LocalPose)))
            return false;
        var definition = new SpatialBodyDefinition(SpatialMotionOwner.Motor, new[] { replacement },
            mass: current.Mass, material: current.Material, collisionLayer: current.CollisionLayer, collisionMask: current.CollisionMask);
        ReconfigureBody((motor.Owner, body), definition, expectedDefinitionRevision, body.LinearVelocity, Vector3.Zero);
        return true;
    }

    private void OnMotorStartup(EntityUid uid, SpatialMotorComponent component, ComponentStartup args)
    {
        component._settings.Validate();
        component._intent.Validate();
        if (!_net.IsServer)
            return;
        if (!TryComp<SpatialBodyComponent>(uid, out var body) || !body.HasDefinition)
            throw new InvalidOperationException("A motor requires its spatial body definition.");
        _motors.Add(uid, component);
        if (body.Running && body.Definition.Owner == SpatialMotionOwner.Motor)
            RefreshBinding(uid);
    }

    private void OnMotorShutdown(EntityUid uid, SpatialMotorComponent component, ComponentShutdown args)
    {
        _motors.Remove(uid);
        _motorIncomingVelocity.Remove(uid);
        component._support = null;
        if (TryComp<SpatialBodyComponent>(uid, out var body) && body.HasDefinition && body.Definition.Owner == SpatialMotionOwner.Motor)
            RemoveBinding(uid);
    }

    private void PrepareMotors(float dt)
    {
        _motorIncomingVelocity.Clear();
        foreach (var (uid, motor) in _motors)
        {
            if (!_bindings.TryGetValue(uid, out var binding) || binding.Paused || MetaData(binding.World).EntityPaused
                || !TryComp<SpatialBodyComponent>(uid, out var body) || body.Definition.Owner != SpatialMotionOwner.Motor)
                continue;
            _motorIncomingVelocity.Add(uid, body.LinearVelocity);
            _worlds[binding.World].PrepareMotorStep(binding.Body, motor.Settings, motor.Intent, motor.Support != null, dt);
        }
    }

    private void PublishMotorSupports()
    {
        var changes = new List<(EntityUid Entity, SpatialMotorSupportChanged Change, Vector3 Incoming)>();
        foreach (var (uid, motor) in _motors)
        {
            SpatialMotorSupport? current = null;
            if (TerminatingOrDeleted(uid))
                continue;
            if (_bindings.TryGetValue(uid, out var binding))
            {
                if (binding.Paused || MetaData(binding.World).EntityPaused)
                    continue;
                if (TryComp<SpatialBodyComponent>(uid, out var body) && body.Definition.Owner == SpatialMotionOwner.Motor
                    && motor.Intent.Mode == SpatialMotorMode.Walking
                    && _worlds[binding.World].FindMotorSupport(binding.Body, motor.Settings) is { } support
                    && support.Distance <= motor.Settings.ContactTolerance * 2
                    && Vector3.Dot(body.LinearVelocity - support.Velocity, support.Normal) <= 0.25f)
                    current = new SpatialMotorSupport(new SpatialQueryCandidate(support.Source.Entity, support.Source.Surface,
                        support.Source.MaterialProfile, support.Source.Revision), support.Point, support.Normal,
                        support.Velocity, support.Distance);
            }
            var previous = motor._support;
            motor._support = current;
            if (previous != current)
                Dirty(uid, motor);
            if (previous?.Source.Entity != current?.Source.Entity || previous?.Source.Surface != current?.Source.Surface
                || previous.HasValue != current.HasValue)
                changes.Add((uid, new SpatialMotorSupportChanged(previous, current),
                    _motorIncomingVelocity.GetValueOrDefault(uid)));
        }
        // All poses/supports are committed before content receives notifications.
        foreach (var (uid, change, incoming) in changes)
        {
            if (TerminatingOrDeleted(uid))
                continue;
            var ev = change;
            RaiseLocalEvent(uid, ref ev);
            if (change.Previous == null && change.Current is { } landed && !TerminatingOrDeleted(uid))
            {
                var landing = new SpatialMotorLanded(landed, incoming - landed.Velocity);
                RaiseLocalEvent(uid, ref landing);
            }
        }
    }

    private void OnMotorGetState(EntityUid uid, SpatialMotorComponent component, ref ComponentGetState args)
    {
        SpatialMotorSupportState? support = null;
        if (component.Support is { } s)
            support = new SpatialMotorSupportState(GetNetEntity(s.Source.Entity), GetNetEntity(s.Source.Surface?.OwnerFrame),
                s.Source.Surface?.Surface, s.Source.MaterialProfile, s.Source.Revision, s.Point, s.Normal, s.Velocity, s.Distance);
        args.State = new SpatialMotorComponentState(component.Settings, component.Intent, support);
    }

    private void OnMotorHandleState(EntityUid uid, SpatialMotorComponent component, ref ComponentHandleState args)
    {
        if (args.Current is not SpatialMotorComponentState state)
            return;
        if (_net.IsServer)
            throw new InvalidOperationException("Server motors cannot be overwritten by client component state.");
        state.Settings.Validate();
        state.Intent.Validate();
        SpatialMotorSupport? support = null;
        if (state.Support is { } s)
        {
            EntityUid? entity = s.Entity is { } e ? EnsureEntity<SpatialMotorComponent>(e, uid) : null;
            SpatialSurfaceReference? surface = s.Frame is { } f && s.Surface is { } id
                ? new SpatialSurfaceReference(EnsureEntity<SpatialMotorComponent>(f, uid), id) : null;
            support = new SpatialMotorSupport(new SpatialQueryCandidate(entity, surface, s.MaterialProfile, s.Revision),
                s.Point, s.Normal, s.Velocity, s.Distance);
        }
        component._settings = state.Settings;
        component._intent = state.Intent;
        component._support = support;
    }

    private static bool IsDynamicOwner(SpatialMotionOwner owner) => owner is SpatialMotionOwner.DynamicBody or SpatialMotionOwner.Motor;

    private void ValidateMotorBody(EntityUid uid, SpatialBodyDefinition definition, Vector3 angular)
    {
        if (!HasComp<SpatialMotorComponent>(uid) || definition.UseGeometry || definition.Colliders.Count != 1
            || definition.Colliders[0].Shape is not SpatialCapsuleShape || definition.Colliders[0].Rotation != Quaternion.Identity
            || angular != Vector3.Zero)
            throw new InvalidOperationException("A motor requires its controller and one upright, rotation-locked capsule.");
    }
}
