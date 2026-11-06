using System;
using System.Numerics;
using Robust.Shared.GameObjects;

namespace Robust.Shared.Spatial.Physics;

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
        var backendDefinition = BuildBackendDefinition(body, copy, revision, out var geometryRevision);
        var paused = MetaData(body).EntityPaused;
        var canonicalState = new SpatialBackendState(_transforms.GetWorldPose(body), linearVelocity, angularVelocity);
        var solverState = paused ? canonicalState with { LinearVelocity = Vector3.Zero, AngularVelocity = Vector3.Zero } : canonicalState;
        Binding? replacement = null;
        if (backendDefinition != null)
        {
            using var prepared = backend.PrepareBody(backendDefinition);
            // Even dormant bodies must have representable mass/inertia and origin
            // velocities. Validation does not insert their shapes into broadphase.
            prepared.ValidateState(canonicalState);
            if (!_containers.IsEntityOrParentInContainer(body))
            {
                var id = prepared.Commit(solverState);
                if (paused && IsDynamicOwner(copy.Owner))
                    backend.SetPaused(id, true);
                replacement = new Binding(body, world.Value, id, copy.UseGeometry, geometryRevision, paused, revision);
            }
        }
        // All validation and fallible shape preparation preceded canonical writes.
        // Existing contacts/connections are removed through their normal lifecycle;
        // canonical connections rebuild against the new body at the next boundary.
        RemoveBinding(body);
        body.Comp._definition = copy;
        body.Comp._definitionRevision = revision;
        body.Comp._linearVelocity = linearVelocity;
        body.Comp._angularVelocity = angularVelocity;
        if (replacement != null)
            _bindings.Add(body, replacement);
        Dirty(body);
    }

    private void ValidateConstraintOwnershipChange(EntityUid body, SpatialBodyDefinition next)
    {
        foreach (var component in _constraintComponents.Values)
        {
            var d = component.Definition;
            if (d.BodyA != body && d.BodyB != body)
                continue;
            var other = d.BodyA == body ? d.BodyB : d.BodyA;
            var otherOwner = TryComp<SpatialBodyComponent>(other, out var participant)
                ? participant.Definition.Owner : SpatialMotionOwner.Static;
            if (!IsDynamicOwner(next.Owner) && !IsDynamicOwner(otherOwner))
                throw new InvalidOperationException("Reconfiguration would leave a connection without a dynamic participant.");
            if (d.Kind == SpatialConstraintKind.Fixed && next.Owner != SpatialMotionOwner.DynamicBody
                && otherOwner != SpatialMotionOwner.DynamicBody)
                throw new InvalidOperationException("A fixed connection requires a participant with angular inertia.");
        }
    }
}
