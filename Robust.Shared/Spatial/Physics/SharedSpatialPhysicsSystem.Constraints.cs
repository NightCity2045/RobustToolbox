using System;
using System.Collections.Generic;
using System.Linq;
using Robust.Shared.GameObjects;
using Robust.Shared.GameStates;
using Robust.Shared.Maths;

namespace Robust.Shared.Spatial.Physics;

public sealed partial class SharedSpatialPhysicsSystem
{
    private readonly Dictionary<EntityUid, SpatialConstraintComponent> _constraintComponents = new();
    private readonly Dictionary<EntityUid, ConstraintBinding> _constraintBindings = new();
    private sealed record ConstraintBinding(EntityUid World, SpatialBackendConstraintId Id,
        SpatialBackendBodyId A, SpatialBackendBodyId B, SpatialConstraintDefinition Definition);

    private void InitializeConstraints()
    {
        SubscribeLocalEvent<SpatialConstraintComponent, ComponentStartup>(OnConstraintStartup);
        SubscribeLocalEvent<SpatialConstraintComponent, ComponentShutdown>(OnConstraintShutdown);
        SubscribeLocalEvent<SpatialConstraintComponent, ComponentGetState>(OnConstraintGetState);
        SubscribeLocalEvent<SpatialConstraintComponent, ComponentHandleState>(OnConstraintHandleState);
    }

    public void ConfigureInitialConstraint(Entity<SpatialConstraintComponent> constraint, SpatialConstraintDefinition definition)
    {
        RequireServer();
        if (MetaData(constraint).EntityLifeStage != EntityLifeStage.PreInit)
            throw new InvalidOperationException("Initial constraints require an uninitialized entity.");
        ValidateConstraintParticipants(constraint, definition);
        constraint.Comp._definition = definition.Copy();
    }

    /// <summary>Replace a connection after validating and preparing its new solver constraint.</summary>
    public void SetConstraint(Entity<SpatialConstraintComponent> constraint, SpatialConstraintDefinition definition)
    {
        RequireServer();
        if (TerminatingOrDeleted(constraint) || !constraint.Comp.Running)
            throw new InvalidOperationException("Constraint replacement requires a running component.");
        ValidateConstraintParticipants(constraint, definition);
        var copy = definition.Copy();
        var prepared = PrepareConstraint(constraint, copy);
        RemoveConstraintBinding(constraint);
        constraint.Comp._definition = copy;
        if (prepared != null)
            _constraintBindings.Add(constraint, prepared);
        Dirty(constraint);
    }

    public void RemoveConstraint(Entity<SpatialConstraintComponent> constraint)
    {
        RequireServer();
        RemComp<SpatialConstraintComponent>(constraint);
    }

    /// <summary>True only for a current server-side solver connection; definitions can be dormant.</summary>
    public bool IsConstraintActive(EntityUid uid)
        => _constraintBindings.TryGetValue(uid, out var binding) && _worlds.TryGetValue(binding.World, out var backend)
            && backend.HasConstraint(binding.Id);

    private void OnConstraintStartup(EntityUid uid, SpatialConstraintComponent component, ComponentStartup args)
    {
        if (!_net.IsServer)
            return;
        ValidateConstraintParticipants(uid, component._definition);
        component._definition = component._definition.Copy();
        _constraintComponents.Add(uid, component);
        // Map loading can initialize the connection before its participants.
        // Binding occurs at the fixed tick boundary, after all entity startup.
    }

    private void OnConstraintShutdown(EntityUid uid, SpatialConstraintComponent component, ComponentShutdown args)
    {
        RemoveConstraintBinding(uid);
        _constraintComponents.Remove(uid);
    }

    private void ValidateConstraintParticipants(EntityUid uid, SpatialConstraintDefinition definition)
    {
        ArgumentNullException.ThrowIfNull(definition);
        definition.Validate();
        var world = SpatialPhysicsOwnership.GetMap(EntityManager, uid);
        if (world == null || !HasComp<SpatialPhysicsWorldComponent>(world.Value)
            || TerminatingOrDeleted(definition.BodyA) || TerminatingOrDeleted(definition.BodyB)
            || SpatialPhysicsOwnership.GetMap(EntityManager, definition.BodyA) != world
            || SpatialPhysicsOwnership.GetMap(EntityManager, definition.BodyB) != world)
            throw new InvalidOperationException("A connection and both bodies must belong to the same spatial physics world.");
        var a = Motion(definition.BodyA);
        var b = Motion(definition.BodyB);
        if (a != SpatialMotionOwner.DynamicBody && b != SpatialMotionOwner.DynamicBody)
            throw new InvalidOperationException("A constraint requires at least one dynamic participant.");

        SpatialMotionOwner Motion(EntityUid entity)
        {
            if (TryComp<SpatialBodyComponent>(entity, out var body) && body.HasDefinition)
                return body.Definition.Owner;
            if (HasComp<SpatialGeometryComponent>(entity))
                return SpatialMotionOwner.Static;
            throw new InvalidOperationException("A connection participant has no spatial body or structural geometry.");
        }
    }

    private void SyncConstraints()
    {
        foreach (var (uid, component) in _constraintComponents)
        {
            var definition = component._definition;
            if (_constraintBindings.TryGetValue(uid, out var current)
                && !MetaData(uid).EntityPaused && SpatialPhysicsOwnership.GetMap(EntityManager, uid) == current.World
                && _bindings.TryGetValue(definition.BodyA, out var a) && _bindings.TryGetValue(definition.BodyB, out var b)
                && a.World == current.World && b.World == current.World && a.Body == current.A && b.Body == current.B
                && _worlds.TryGetValue(current.World, out var activeBackend) && activeBackend.HasConstraint(current.Id))
                continue;
            var next = PrepareConstraint(uid, definition);
            RemoveConstraintBinding(uid);
            if (next != null)
                _constraintBindings.Add(uid, next);
        }
    }

    private ConstraintBinding? PrepareConstraint(EntityUid uid, SpatialConstraintDefinition definition)
    {
        if (MetaData(uid).EntityPaused || TerminatingOrDeleted(uid)
            || !_bindings.TryGetValue(definition.BodyA, out var a) || !_bindings.TryGetValue(definition.BodyB, out var b)
            || a.World != b.World || SpatialPhysicsOwnership.GetMap(EntityManager, uid) != a.World
            || !_worlds.TryGetValue(a.World, out var backend))
            return null;
        var dynamicA = !a.Paused && TryComp<SpatialBodyComponent>(definition.BodyA, out var bodyA)
            && bodyA.Definition.Owner == SpatialMotionOwner.DynamicBody;
        var dynamicB = !b.Paused && TryComp<SpatialBodyComponent>(definition.BodyB, out var bodyB)
            && bodyB.Definition.Owner == SpatialMotionOwner.DynamicBody;
        if (!dynamicA && !dynamicB)
            return null;
        var id = backend.AddConstraint(definition.ToBackend(a.Body, b.Body));
        return new ConstraintBinding(a.World, id, a.Body, b.Body, definition);
    }

    private void RemoveConstraintBinding(EntityUid uid)
    {
        if (_constraintBindings.Remove(uid, out var binding) && _worlds.TryGetValue(binding.World, out var backend)
            && backend.HasConstraint(binding.Id))
            backend.RemoveConstraint(binding.Id);
    }

    private void RemoveConstraintsReferencing(EntityUid body)
    {
        foreach (var (uid, component) in _constraintComponents.ToArray())
        {
            if (component.Definition.BodyA != body && component.Definition.BodyB != body)
                continue;
            RemoveConstraintBinding(uid);
            if (!TerminatingOrDeleted(uid))
                RemComp<SpatialConstraintComponent>(uid);
        }
    }

    private void OnConstraintGetState(EntityUid uid, SpatialConstraintComponent component, ref ComponentGetState args)
    {
        var d = component.Definition;
        args.State = new SpatialConstraintComponentState(GetNetEntity(d.BodyA), GetNetEntity(d.BodyB), d.Kind,
            d.LocalAnchorA, d.LocalAnchorB, d.RelativeOffset, d.RelativeRotation, d.MinimumDistance, d.MaximumDistance,
            d.SpringFrequency, d.Damping, d.CollideConnected);
    }

    private void OnConstraintHandleState(EntityUid uid, SpatialConstraintComponent component, ref ComponentHandleState args)
    {
        if (args.Current is not SpatialConstraintComponentState state)
            return;
        if (_net.IsServer)
            throw new InvalidOperationException("Server constraints cannot be overwritten by a client component state.");
        var a = EnsureEntity<SpatialConstraintComponent>(state.A, uid);
        var b = EnsureEntity<SpatialConstraintComponent>(state.B, uid);
        component._definition = new SpatialConstraintDefinition(a, b, state.Kind, state.AnchorA, state.AnchorB,
            new RigidTransform3(state.RelativeOffset, state.RelativeRotation), state.Minimum, state.Maximum,
            state.Frequency, state.Damping, state.CollideConnected);
    }
}
