using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using BepuPhysics;
using BepuPhysics.Constraints;

namespace Robust.Shared.Spatial.Physics;

internal sealed partial class BepuSpatialPhysicsBackend
{
    private readonly Dictionary<SpatialBackendConstraintId, ConstraintEntry> _constraints = new();
    private readonly Dictionary<SpatialBackendBodyId, HashSet<SpatialBackendConstraintId>> _bodyConstraints = new();
    private readonly Dictionary<(SpatialBackendBodyId, SpatialBackendBodyId), int> _disabledContactPairs = new();
    private ulong _nextConstraint;

    private sealed record ConstraintEntry(SpatialBackendConstraintDefinition Definition, ConstraintHandle Handle,
        BodyHandle BodyA, BodyHandle BodyB, bool AnchorA, bool AnchorB);

    public int ConstraintCount => _constraints.Count;
    internal bool HasConstraint(SpatialBackendConstraintId id) => _constraints.ContainsKey(id);

    public SpatialBackendConstraintId AddConstraint(SpatialBackendConstraintDefinition definition)
    {
        ThrowIfDisposed();
        ArgumentNullException.ThrowIfNull(definition);
        definition.Validate();
        var a = _entries[definition.BodyA];
        var b = _entries[definition.BodyB];
        if ((a.Definition.Motion != SpatialBackendMotion.Dynamic || a.Paused)
            && (b.Definition.Motion != SpatialBackendMotion.Dynamic || b.Paused))
            throw new InvalidOperationException("A constraint requires at least one dynamic participant.");
        if (definition.Kind == SpatialConstraintKind.Fixed && !CanRotate(a) && !CanRotate(b))
            throw new InvalidOperationException("A fixed connection requires a participant with angular inertia.");
        var localA = definition.LocalAnchorA - a.Center;
        var localB = definition.LocalAnchorB - b.Center;
        var relative = definition.RelativePose;
        var offset = relative.Position + Vector3.Transform(b.Center, relative.Rotation) - a.Center;
        if (!SpatialBackendBodyDefinition.Finite(localA) || !SpatialBackendBodyDefinition.Finite(localB)
            || !SpatialBackendBodyDefinition.Finite(offset))
            throw new ArgumentException("Constraint attachment exceeds the finite solver range.");
        var id = new SpatialBackendConstraintId(checked(++_nextConstraint));
        var anchorA = a.Definition.Motion == SpatialBackendMotion.Static;
        var anchorB = b.Definition.Motion == SpatialBackendMotion.Static;
        var handleA = a.Body;
        var handleB = b.Body;
        var allocatedA = false;
        var allocatedB = false;
        try
        {
            if (anchorA)
            {
                handleA = CreateAnchor(a);
                allocatedA = true;
            }
            if (anchorB)
            {
                handleB = CreateAnchor(b);
                allocatedB = true;
            }
            var spring = new SpringSettings(definition.SpringFrequency, definition.Damping);
            var handle = definition.Kind switch
            {
                SpatialConstraintKind.Fixed => _simulation.Solver.Add(handleA, handleB,
                    new Weld { LocalOffset = offset, LocalOrientation = relative.Rotation, SpringSettings = spring }),
                SpatialConstraintKind.Point => _simulation.Solver.Add(handleA, handleB,
                    new BallSocket { LocalOffsetA = localA, LocalOffsetB = localB, SpringSettings = spring }),
                SpatialConstraintKind.DistanceRange => _simulation.Solver.Add(handleA, handleB,
                    new DistanceLimit { LocalOffsetA = localA, LocalOffsetB = localB,
                        MinimumDistance = definition.MinimumDistance, MaximumDistance = definition.MaximumDistance,
                        SpringSettings = spring }),
                _ => throw new ArgumentOutOfRangeException(nameof(definition))
            };
            _constraints.Add(id, new ConstraintEntry(definition, handle, handleA, handleB, anchorA, anchorB));
            Track(a.Id);
            Track(b.Id);
            if (!definition.CollideConnected)
            {
                var pair = ContactPair(a.Id, b.Id);
                _disabledContactPairs.TryGetValue(pair, out var count);
                _disabledContactPairs[pair] = count + 1;
            }
            return id;
        }
        catch
        {
            if (allocatedB)
                _simulation.Bodies.Remove(handleB);
            if (allocatedA)
                _simulation.Bodies.Remove(handleA);
            throw;
        }

        void Track(SpatialBackendBodyId body)
        {
            if (!_bodyConstraints.TryGetValue(body, out var attached))
                _bodyConstraints.Add(body, attached = new HashSet<SpatialBackendConstraintId>());
            attached.Add(id);
        }
    }

    private static bool CanRotate(Entry entry) => entry.Definition.Motion == SpatialBackendMotion.Dynamic
        && !entry.Definition.LockRotation && !entry.Paused;

    public void RemoveConstraint(SpatialBackendConstraintId id)
    {
        ThrowIfDisposed();
        var entry = _constraints[id];
        // Removing a connection can restore collisions or release a sleeping
        // assembly. Wake both islands before changing their solver graph.
        var bodyA = _simulation.Bodies.GetBodyReference(entry.BodyA);
        var bodyB = _simulation.Bodies.GetBodyReference(entry.BodyB);
        bodyA.Awake = true;
        bodyB.Awake = true;
        _simulation.Solver.Remove(entry.Handle);
        if (entry.AnchorA)
            _simulation.Bodies.Remove(entry.BodyA);
        if (entry.AnchorB)
            _simulation.Bodies.Remove(entry.BodyB);
        var definition = entry.Definition;
        Untrack(definition.BodyA);
        Untrack(definition.BodyB);
        _constraints.Remove(id);
        if (!definition.CollideConnected)
        {
            var pair = ContactPair(definition.BodyA, definition.BodyB);
            if (--_disabledContactPairs[pair] == 0)
                _disabledContactPairs.Remove(pair);
        }

        void Untrack(SpatialBackendBodyId body)
        {
            var attached = _bodyConstraints[body];
            attached.Remove(id);
            if (attached.Count == 0)
                _bodyConstraints.Remove(body);
        }
    }

    private BodyHandle CreateAnchor(Entry entry)
    {
        // BEPU constraints connect BodyHandles. A static collidable gets a private,
        // shapeless kinematic anchor; this is derived solver data, not a new entity.
        var pose = _simulation.Statics.GetStaticReference(entry.Static).Pose;
        return _simulation.Bodies.Add(BodyDescription.CreateKinematic(pose, default, 0.01f));
    }

    private void UpdateConstraintAnchors(Entry entry)
    {
        if (entry.Definition.Motion != SpatialBackendMotion.Static || !_bodyConstraints.TryGetValue(entry.Id, out var attached))
            return;
        var pose = _simulation.Statics.GetStaticReference(entry.Static).Pose;
        foreach (var id in attached)
        {
            var constraint = _constraints[id];
            var body = _simulation.Bodies.GetBodyReference(constraint.Definition.BodyA == entry.Id ? constraint.BodyA : constraint.BodyB);
            body.Awake = true;
            body.Pose = pose;
        }
    }

    private void RemoveBodyConstraints(SpatialBackendBodyId body)
    {
        if (_bodyConstraints.TryGetValue(body, out var attached))
            foreach (var id in attached.ToArray())
                RemoveConstraint(id);
    }

    private static (SpatialBackendBodyId, SpatialBackendBodyId) ContactPair(SpatialBackendBodyId a, SpatialBackendBodyId b)
        => a.Value < b.Value ? (a, b) : (b, a);
}
