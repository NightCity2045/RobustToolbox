using System;
using System.Numerics;
using BepuPhysics;

namespace Robust.Shared.Spatial.Physics;

internal sealed partial class BepuSpatialPhysicsBackend
{
    public void ApplyImpulse(SpatialBackendBodyId id, Vector3 worldImpulse, Vector3 worldPoint)
    {
        ThrowIfDisposed();
        if (!SpatialBackendBodyDefinition.Finite(worldImpulse) || !SpatialBackendBodyDefinition.Finite(worldPoint))
            throw new ArgumentException("Impulse and application point must be finite.");
        var entry = _entries[id];
        if (entry.Definition.Motion != SpatialBackendMotion.Dynamic)
            throw new InvalidOperationException("Only a dynamic body accepts an impulse.");
        var body = _simulation.Bodies.GetBodyReference(entry.Body);
        var velocity = body.Velocity;
        var offset = worldPoint - body.Pose.Position;
        if (!SpatialBackendBodyDefinition.Finite(offset))
            throw new ArgumentException("Impulse offset exceeds the finite solver range.");
        // Evaluate on a copy before waking or modifying the solver body. Inertia
        // is transformed by BEPU at the center of mass, not the entity origin.
        var inertia = body.LocalInertia;
        var pose = body.Pose;
        BodyReference.ApplyImpulse(worldImpulse, offset, ref inertia, ref pose, ref velocity);
        var originVelocity = velocity.Linear - Vector3.Cross(velocity.Angular,
            Vector3.Transform(entry.Center, body.Pose.Orientation));
        if (!SpatialBackendBodyDefinition.Finite(velocity.Linear) || !SpatialBackendBodyDefinition.Finite(velocity.Angular)
            || !SpatialBackendBodyDefinition.Finite(originVelocity))
            throw new ArgumentException("Impulse result exceeds the finite solver range.");
        if (worldImpulse == Vector3.Zero)
            return;
        body.Awake = true;
        body.Velocity = velocity;
    }
}
