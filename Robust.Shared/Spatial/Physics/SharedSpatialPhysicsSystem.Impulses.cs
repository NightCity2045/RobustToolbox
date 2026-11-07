using System;
using System.Numerics;
using Robust.Shared.GameObjects;

namespace Robust.Shared.Spatial.Physics;

/// <summary>Authoritative notification after canonical velocities have been updated.</summary>
[ByRefEvent]
public readonly record struct SpatialImpulseApplied(Vector3 WorldPoint, Vector3 WorldImpulse,
    Vector3 LinearVelocity, Vector3 AngularVelocity);

public sealed partial class SharedSpatialPhysicsSystem
{
    /// <summary>
    /// Apply world-space momentum at a world-space point. Only an active, unpaused
    /// dynamic body or motor is eligible; contained/static bodies cannot receive a
    /// hidden physical impulse. Zero momentum has no effect or event.
    /// </summary>
    public void ApplyImpulse(Entity<SpatialBodyComponent> body, Vector3 worldImpulse, Vector3 worldPoint)
    {
        RequireMotorAuthority(body);
        if (!_net.IsServer)
        {
            if (!SpatialBackendBodyDefinition.Finite(worldImpulse) || !SpatialBackendBodyDefinition.Finite(worldPoint))
                throw new ArgumentException("An impulse requires a finite point and momentum.");
            // The selected capsule has locked rotation. Replay writes the same
            // canonical velocity and never emits authoritative impact events.
            SetVelocity(body, body.Comp.LinearVelocity + worldImpulse / body.Comp.Definition.Mass, Vector3.Zero);
            return;
        }
        if (body.Comp.Definition.Owner == SpatialMotionOwner.AnchoredCompoundChild)
        {
            if (!_compoundOwners.TryGetValue(body, out var root) || !TryComp<SpatialBodyComponent>(root, out var owner))
                throw new InvalidOperationException("A compound impulse requires an active dynamic root.");
            ApplyImpulse((root, owner), worldImpulse, worldPoint);
            return;
        }
        if (TerminatingOrDeleted(body) || !IsDynamicOwner(body.Comp.Definition.Owner)
            || !_bindings.TryGetValue(body, out var binding) || binding.Paused || MetaData(binding.World).EntityPaused)
            throw new InvalidOperationException("An impulse requires an active, unpaused dynamic body.");
        var backend = _worlds[binding.World];
        backend.ApplyImpulse(binding.Body, worldImpulse, worldPoint);
        if (worldImpulse == Vector3.Zero)
            return;
        var state = backend.GetState(binding.Body);
        body.Comp._linearVelocity = state.LinearVelocity;
        body.Comp._angularVelocity = state.AngularVelocity;
        Dirty(body);
        var ev = new SpatialImpulseApplied(worldPoint, worldImpulse, state.LinearVelocity, state.AngularVelocity);
        RaiseLocalEvent(body, ref ev);
    }

    /// <summary>World velocity of a point fixed to this body's canonical pose.</summary>
    public Vector3 GetVelocityAtWorldPoint(Entity<SpatialBodyComponent> body, Vector3 worldPoint)
    {
        if (TerminatingOrDeleted(body) || !SpatialBackendBodyDefinition.Finite(worldPoint))
            throw new ArgumentException("Point velocity requires a live body and finite point.");
        if (body.Comp.Definition.Owner == SpatialMotionOwner.AnchoredCompoundChild)
        {
            var root = FindCompoundRoot(body) ?? throw new InvalidOperationException("The compound child has no physical root.");
            return TryComp<SpatialBodyComponent>(root, out var owner) ? GetVelocityAtWorldPoint((root, owner), worldPoint) : Vector3.Zero;
        }
        var pose = _transforms.GetWorldPose(body);
        var velocity = body.Comp.LinearVelocity + Vector3.Cross(body.Comp.AngularVelocity, worldPoint - pose.Position);
        if (!SpatialBackendBodyDefinition.Finite(velocity))
            throw new ArgumentException("Point velocity exceeds the finite spatial range.");
        return velocity;
    }
}
