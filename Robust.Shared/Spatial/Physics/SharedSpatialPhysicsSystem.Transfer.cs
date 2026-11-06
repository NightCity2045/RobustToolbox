using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using Robust.Shared.GameObjects;
using Robust.Shared.Maths;

namespace Robust.Shared.Spatial.Physics;

[ByRefEvent]
public readonly record struct SpatialGraphTransferred(EntityUid SourceWorld, EntityUid DestinationWorld,
    IReadOnlyList<EntityUid> Entities, RigidTransform3 WorldMapping);

public sealed partial class SharedSpatialPhysicsSystem
{
    private bool _transferring;

    /// <summary>
    /// Transfer explicitly selected, disjoint hierarchies as one closed physical graph.
    /// Mapping transforms old world poses into destination world poses and rotates
    /// world velocities once. Content selects passengers/cargo and validates its
    /// own gameplay links before calling; the engine never guesses that policy.
    /// </summary>
    public void TransferGraph(IReadOnlyCollection<EntityUid> roots, EntityUid destination, RigidTransform3 worldMapping)
    {
        RequireServer();
        ArgumentNullException.ThrowIfNull(roots);
        if (_updating || _committing || _transferring)
            throw new InvalidOperationException("A graph transfer cannot reenter a physical commit or step.");
        _transferring = true;
        try { TransferGraphCore(roots, destination, worldMapping); }
        finally { _transferring = false; }
    }

    private void TransferGraphCore(IReadOnlyCollection<EntityUid> roots, EntityUid destination, RigidTransform3 mapping)
    {
        var hierarchy = _transforms.PrepareSpatialHierarchyTransfer(roots, destination, mapping);
        if (!_worlds.TryGetValue(hierarchy.Source, out var sourceBackend) || !_worlds.TryGetValue(destination, out var targetBackend))
            throw new InvalidOperationException("Both transfer worlds require active spatial physics profiles.");
        var members = hierarchy.Nodes.Select(n => n.Entity.Owner).ToHashSet();
        ValidateTransferClosure(members, hierarchy.Source);
        var bodies = new List<(EntityUid Entity, SpatialBodyComponent Body, Vector3 Linear, Vector3 Angular)>();
        foreach (var uid in members)
        {
            if (!TryComp<SpatialBodyComponent>(uid, out var body)) continue;
            var linear = mapping.TransformVector(body.LinearVelocity);
            var angular = mapping.TransformVector(body.AngularVelocity);
            ValidateDefinition(uid, body.Definition, linear, angular);
            bodies.Add((uid, body, linear, angular));
        }
        var motors = new List<(EntityUid Entity, SpatialMotorComponent Motor, SpatialMotorIntent Intent,
            SpatialMotorSupport? Support, SpatialMotorSupportChanged Change)>();
        foreach (var (uid, motor) in _motors)
        {
            var moved = members.Contains(uid);
            var before = motor.Support;
            var supportMoved = before is { } support
                && (support.Source.Entity is { } source && members.Contains(source)
                    || support.Source.Surface is { } surface && members.Contains(surface.OwnerFrame));
            if (!moved && !supportMoved) continue;
            var intent = moved ? motor.Intent with { Velocity = mapping.TransformVector(motor.Intent.Velocity) } : motor.Intent;
            intent.Validate();
            SpatialMotorSupport? next = moved && supportMoved && before is { } current
                ? current with { Point = mapping.TransformPoint(current.Point),
                    Normal = mapping.TransformVector(current.Normal), Velocity = mapping.TransformVector(current.Velocity) } : null;
            motors.Add((uid, motor, intent, next, new SpatialMotorSupportChanged(before, next)));
        }
        var replacements = new Dictionary<EntityUid, Binding>();
        var constraints = new Dictionary<EntityUid, ConstraintBinding>();
        var installed = false;
        try
        {
            // Destination allocations/shape/inertia/state validation occur while
            // the complete canonical graph and old bodies are still untouched.
            foreach (var uid in members.Order())
            {
                if (!_bindings.TryGetValue(uid, out var old)) continue;
                if (old.World != hierarchy.Source || old.Compound != null && !CompoundStillCurrent(old.Compound))
                    throw new InvalidOperationException("A physical binding is stale before transfer.");
                var definition = sourceBackend.GetDefinition(old.Body);
                var hasBody = TryComp<SpatialBodyComponent>(uid, out var body);
                var paused = hasBody && hierarchy.Paused;
                var state = new SpatialBackendState(mapping.Compose(_transforms.GetWorldPose(uid)),
                    paused ? Vector3.Zero : mapping.TransformVector(body?.LinearVelocity ?? Vector3.Zero),
                    paused ? Vector3.Zero : mapping.TransformVector(body?.AngularVelocity ?? Vector3.Zero));
                using var prepared = targetBackend.PrepareBody(definition);
                prepared.ValidateState(state);
                var id = prepared.Commit(state);
                replacements.Add(uid, old with { World = destination, Body = id, Paused = paused });
                if (paused && definition.Motion == SpatialBackendMotion.Dynamic) targetBackend.SetPaused(id, true);
            }
            if (!hierarchy.Paused)
            {
                foreach (var uid in members)
                {
                    if (!_constraintComponents.TryGetValue(uid, out var component)) continue;
                    var d = component.Definition;
                    var rootA = ResolvePhysicalOwner(d.BodyA);
                    var rootB = ResolvePhysicalOwner(d.BodyB);
                    if (rootA == null || rootB == null || !replacements.TryGetValue(rootA.Value, out var a)
                        || !replacements.TryGetValue(rootB.Value, out var b)) continue; // Dormant contained participant.
                    var relativeA = CompoundRelativePose(d.BodyA, rootA.Value);
                    var relativeB = CompoundRelativePose(d.BodyB, rootB.Value);
                    var resolved = d.ToBackend(a.Body, b.Body) with
                    {
                        LocalAnchorA = relativeA.TransformPoint(d.LocalAnchorA),
                        LocalAnchorB = relativeB.TransformPoint(d.LocalAnchorB),
                        RelativePose = relativeA.Compose(d.RelativePose).Compose(relativeB.Inverse())
                    };
                    var id = targetBackend.AddConstraint(resolved);
                    constraints.Add(uid, new ConstraintBinding(destination, id, a.Body, b.Body, d));
                }
            }
            _transforms.ValidateSpatialHierarchyTransfer(hierarchy);
            var supportChanges = new List<(EntityUid Entity, SpatialMotorSupportChanged Change)>();
            _transforms.CommitSpatialHierarchyTransfer(hierarchy, () =>
            {
                foreach (var uid in members) RemoveConstraintBinding(uid);
                foreach (var uid in members) RemoveBinding(uid);
                foreach (var (uid, body, linear, angular) in bodies)
                {
                    body._linearVelocity = linear;
                    body._angularVelocity = angular;
                    Dirty(uid, body);
                }
                foreach (var binding in replacements.Values) InstallBinding(binding);
                foreach (var (uid, binding) in constraints) _constraintBindings.Add(uid, binding);
                foreach (var (uid, motor, intent, support, change) in motors)
                {
                    motor._intent = intent;
                    motor._support = support;
                    _motorIncomingVelocity.Remove(uid);
                    Dirty(uid, motor);
                    if (change.Previous != change.Current) supportChanges.Add((uid, change));
                }
                installed = true;
            });
            foreach (var (uid, change) in supportChanges)
            {
                if (TerminatingOrDeleted(uid)) continue;
                var ev = change;
                RaiseLocalEvent(uid, ref ev);
            }
            var completed = new SpatialGraphTransferred(hierarchy.Source, destination,
                Array.AsReadOnly(members.Order().ToArray()), mapping);
            RaiseLocalEvent(ref completed);
        }
        finally
        {
            if (!installed)
            {
                foreach (var binding in constraints.Values) targetBackend.RemoveConstraint(binding.Id);
                foreach (var binding in replacements.Values) targetBackend.RemoveBody(binding.Body);
            }
        }
    }

    private void ValidateTransferClosure(HashSet<EntityUid> members, EntityUid sourceWorld)
    {
        foreach (var (uid, component) in _constraintComponents)
        {
            var d = component.Definition;
            var count = (members.Contains(uid) ? 1 : 0) + (members.Contains(d.BodyA) ? 1 : 0) + (members.Contains(d.BodyB) ? 1 : 0);
            if (count != 0 && count != 3)
                throw new InvalidOperationException("Transfer must include each physical connection and both participants.");
        }
        // Geometry source references can cross transform subtrees. Check both
        // incoming and outgoing links, including non-colliding semantic records.
        var query = EntityQueryEnumerator<SpatialGeometryComponent>();
        while (query.MoveNext(out var frame, out _))
        {
            if (SpatialPhysicsOwnership.GetMap(EntityManager, frame) != sourceWorld) continue;
            var snapshot = _geometry.CaptureSnapshot(frame);
            foreach (var entity in snapshot.Boundaries.Values.Select(b => b.Entity).Concat(snapshot.Slabs.Values.Select(s => s.Entity)))
                if (entity is { } uid && members.Contains(frame) != members.Contains(uid))
                    throw new InvalidOperationException("Transfer would split a geometry frame from its semantic source entity.");
        }
    }
}
