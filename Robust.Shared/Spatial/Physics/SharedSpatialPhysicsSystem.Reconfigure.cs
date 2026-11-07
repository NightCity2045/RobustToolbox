using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using Robust.Shared.GameObjects;

namespace Robust.Shared.Spatial.Physics;

/// <summary>
/// Broadcast after an authoritative body reconfiguration commits. Consumers can
/// read the new canonical definition (for example its mass). This is not raised
/// by client state restoration or prediction replay.
/// </summary>
[ByRefEvent]
public readonly record struct SpatialBodyDefinitionChanged(EntityUid Entity);

public sealed partial class SharedSpatialPhysicsSystem
{
    /// <summary>
    /// Atomically replace a body's physical definition. The caller explicitly
    /// supplies canonical velocities and the expected definition revision; no
    /// implicit momentum/gameplay policy is applied. Space/permission checks belong
    /// to the calling motor, placement or gameplay coordinator.
    /// </summary>
    public void ReconfigureBody(Entity<SpatialBodyComponent> body, SpatialBodyDefinition definition,
        ulong expectedDefinitionRevision, Vector3 linearVelocity, Vector3 angularVelocity)
    {
        RequireServer();
        if (TerminatingOrDeleted(body) || !body.Comp.Running || body.Comp.DefinitionRevision != expectedDefinitionRevision)
            throw new InvalidOperationException("Body definition is stale or unavailable.");
        var revision = checked(expectedDefinitionRevision + 1);
        ValidateDefinition(body, definition, linearVelocity, angularVelocity);
        ValidateGeometryOwnership(body, definition);
        ValidateConstraintOwnershipChange(body, definition);
        var world = SpatialPhysicsOwnership.GetMap(EntityManager, body);
        if (world == null || !_worlds.TryGetValue(world.Value, out var backend))
            throw new InvalidOperationException("Body reconfiguration requires an owning spatial physics world.");
        var copy = definition.Copy();
        var overrides = new Dictionary<EntityUid, CompoundBodyOverride> { [body] = new(copy, revision) };
        var roots = new HashSet<EntityUid>();
        if (ResolvePhysicalOwner(body) is { } previousRoot) roots.Add(previousRoot);
        var nextRoot = copy.Owner == SpatialMotionOwner.AnchoredCompoundChild
            ? FindCompoundRoot(body) ?? throw new InvalidOperationException("The new compound owner is unavailable.") : body.Owner;
        roots.Add(nextRoot);
        if (nextRoot != body.Owner) roots.Remove(body.Owner);
        var prepared = new List<(EntityUid Root, CompoundCapture Capture, BepuSpatialPhysicsBackend.PreparedBody? Shape,
            SpatialBackendState State, bool Paused, bool Insert, ulong DefinitionRevision, bool Geometry)>();
        var replacements = new List<Binding>();
        var canonicalStarted = false;
        try
        {
            foreach (var root in roots.Order())
            {
                TryComp<SpatialBodyComponent>(root, out var participant);
                var rootDefinition = root == body.Owner ? copy : participant?.Definition
                    ?? new SpatialBodyDefinition(SpatialMotionOwner.Static, useGeometry: true);
                var rootRevision = root == body.Owner ? revision : participant?.DefinitionRevision ?? 0;
                var linear = root == body.Owner ? linearVelocity : participant?.LinearVelocity ?? Vector3.Zero;
                var angular = root == body.Owner ? angularVelocity : participant?.AngularVelocity ?? Vector3.Zero;
                ValidateDefinition(root, rootDefinition, linear, angular);
                ValidateGeometryOwnership(root, rootDefinition);
                var capture = CaptureCompound(root, rootDefinition, rootRevision, bodyOverrides: overrides);
                var shape = capture.Definition == null ? null : backend.PrepareBody(capture.Definition);
                var paused = participant != null && MetaData(root).EntityPaused;
                var canonical = new SpatialBackendState(_transforms.GetWorldPose(root), linear, angular);
                var state = paused ? canonical with { LinearVelocity = Vector3.Zero, AngularVelocity = Vector3.Zero } : canonical;
                prepared.Add((root, capture, shape, state, paused, !_containers.IsEntityOrParentInContainer(root), rootRevision, rootDefinition.UseGeometry));
                shape?.ValidateState(canonical);
                if (!CompoundStillCurrent(capture)) throw new InvalidOperationException("Compound changed during body preparation.");
            }
            foreach (var plan in prepared)
            {
                if (!plan.Insert || plan.Shape == null) continue;
                var id = plan.Shape.Commit(plan.State);
                var geometryRevision = TryComp<SpatialGeometryComponent>(plan.Root, out var geometry) ? geometry.Revision : 0;
                replacements.Add(new Binding(plan.Root, world.Value, id, plan.Geometry, geometryRevision, plan.Paused,
                    plan.DefinitionRevision, plan.Capture));
                if (plan.Paused && plan.Capture.Definition!.Motion == SpatialBackendMotion.Dynamic)
                    backend.SetPaused(id, true);
            }
            // Every affected hull has been prepared and validated before either
            // root loses its old body or the child's canonical owner changes.
            canonicalStarted = true;
            RemoveBinding(body);
            foreach (var root in roots) RemoveBinding(root);
            body.Comp._definition = copy;
            body.Comp._definitionRevision = revision;
            body.Comp._linearVelocity = linearVelocity;
            body.Comp._angularVelocity = angularVelocity;
            foreach (var replacement in replacements) InstallBinding(replacement);
        }
        catch
        {
            if (!canonicalStarted)
                foreach (var replacement in replacements) backend.RemoveBody(replacement.Body);
            throw;
        }
        finally { foreach (var plan in prepared) plan.Shape?.Dispose(); }
        Dirty(body);
        var changed = new SpatialBodyDefinitionChanged(body);
        RaiseLocalEvent(ref changed);
    }

    private void ValidateConstraintOwnershipChange(EntityUid body, SpatialBodyDefinition next)
    {
        var overrides = new Dictionary<EntityUid, CompoundBodyOverride> { [body] = new(next, 0) };
        foreach (var component in _constraintComponents.Values)
        {
            var d = component.Definition;
            var oldA = ResolvePhysicalOwner(d.BodyA);
            var oldB = ResolvePhysicalOwner(d.BodyB);
            var nextA = ResolvePhysicalOwner(d.BodyA, overrides);
            var nextB = ResolvePhysicalOwner(d.BodyB, overrides);
            if (oldA == nextA && oldB == nextB && oldA != body && oldB != body) continue;
            if (nextA == null || nextB == null || nextA == nextB)
                throw new InvalidOperationException("Reconfiguration would remove or merge a connection's physical participants.");
            var a = Motion(nextA.Value);
            var b = Motion(nextB.Value);
            if (!IsDynamicOwner(a) && !IsDynamicOwner(b))
                throw new InvalidOperationException("Reconfiguration would leave a connection without a dynamic participant.");
            if (d.Kind == SpatialConstraintKind.Fixed && a != SpatialMotionOwner.DynamicBody && b != SpatialMotionOwner.DynamicBody)
                throw new InvalidOperationException("A fixed connection requires a participant with angular inertia.");
        }
        return;

        SpatialMotionOwner Motion(EntityUid entity) => entity == body ? next.Owner
            : TryComp<SpatialBodyComponent>(entity, out var participant) ? participant.Definition.Owner : SpatialMotionOwner.Static;
    }
}
