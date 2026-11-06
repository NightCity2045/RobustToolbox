using System;
using System.Linq;
using System.Numerics;
using Robust.Shared.Maths;

namespace Robust.Shared.Spatial.Physics;

internal sealed partial class BepuSpatialPhysicsBackend
{
    private bool MotorCollides(Entry motor, SpatialBackendBodyId other, SpatialBackendPart part)
    {
        var target = _entries[other].Definition;
        return (motor.Definition.CollisionLayer & (part.CollisionMask ?? target.CollisionMask)) != 0
            && ((part.CollisionLayer ?? target.CollisionLayer) & motor.Definition.CollisionMask) != 0
            && !_disabledContactPairs.ContainsKey(ContactPair(motor.Id, other));
    }

    private Vector3 PointVelocity(SpatialBackendBodyId id, Vector3 point)
    {
        var state = GetState(id);
        return state.LinearVelocity + Vector3.Cross(state.AngularVelocity, point - state.Pose.Position);
    }

    internal SpatialBackendMotorSupport? FindMotorSupport(SpatialBackendBodyId id, SpatialMotorSettings settings)
    {
        var entry = _entries[id];
        var part = entry.Definition.Parts.Single();
        var pose = GetState(id).Pose.Compose(part.LocalPose);
        var up = pose.TransformVector(Vector3.UnitZ);
        var lift = settings.ContactTolerance * 4;
        var distance = lift + settings.SupportDistance;
        var start = new RigidTransform3(pose.Position + up * lift, pose.Rotation);
        SpatialBackendMotorSupport? closest = null;
        foreach (var hit in ShapeCast(part.Shape, start, -up * distance, default, entry.Definition.CollisionMask, id))
        {
            if (!MotorCollides(entry, hit.Body, hit.Source) || hit.Normal is not { } normal || hit.Point is not { } point
                || Vector3.Dot(normal, up) < MathF.Cos(settings.MaximumSlope))
                continue;
            var gap = hit.Fraction * distance - lift;
            if (closest == null || gap < closest.Value.Distance)
                closest = new SpatialBackendMotorSupport(hit.Body, hit.Source, point, normal, PointVelocity(hit.Body, point), gap);
        }
        // An embedded start has no sweep witness. Use actual manifolds instead
        // of treating an overlap as a fictitious horizontal floor.
        if (closest == null)
            foreach (var overlap in OverlapContacts(part.Shape, pose, entry.Definition.CollisionMask, id))
            {
                if (!MotorCollides(entry, overlap.Body, overlap.Source))
                    continue;
                foreach (var contact in overlap.Contacts)
                    if (Vector3.Dot(contact.Normal, up) >= MathF.Cos(settings.MaximumSlope)
                        && (closest == null || -contact.Depth < closest.Value.Distance))
                        closest = new SpatialBackendMotorSupport(overlap.Body, overlap.Source, contact.Point,
                            contact.Normal, PointVelocity(overlap.Body, contact.Point), -contact.Depth);
            }
        return closest;
    }

    /// <summary>
    /// Bounded control impulses precede the common physics step. Contacts/CCD solve
    /// the actual trajectory. Support reactions act on dynamic platforms as well.
    /// Only checked step/snap corrections change the pre-step pose.
    /// </summary>
    internal void PrepareMotorStep(SpatialBackendBodyId id, SpatialMotorSettings settings,
        SpatialMotorIntent intent, bool wasSupported, float dt)
    {
        settings.Validate();
        intent.Validate();
        if (!float.IsFinite(dt) || dt <= 0)
            throw new ArgumentOutOfRangeException(nameof(dt));
        var entry = _entries[id];
        if (entry.Paused || intent.Mode == SpatialMotorMode.Disabled)
            return;
        var state = GetState(id);
        var part = entry.Definition.Parts.Single();
        var up = state.Pose.Compose(part.LocalPose).TransformVector(Vector3.UnitZ);
        var support = intent.Mode == SpatialMotorMode.Walking ? FindMotorSupport(id, settings) : null;
        if (support is { } ground)
        {
            var departing = Vector3.Dot(state.LinearVelocity - ground.Velocity, ground.Normal) > 0.25f;
            if (departing || ground.Distance > settings.ContactTolerance * 2 && !wasSupported)
                support = null;
            else if (ground.Distance > settings.ContactTolerance * 2)
            {
                var downward = -up * ground.Distance;
                if (!MotorPathBlocked(entry, state.Pose.Compose(part.LocalPose), downward, settings.ContactTolerance))
                {
                    state = state with { Pose = new RigidTransform3(state.Pose.Position + downward, state.Pose.Rotation) };
                    SetState(id, state);
                }
            }
        }
        Vector3 correction;
        float acceleration;
        if (intent.Mode == SpatialMotorMode.Free)
        {
            correction = intent.Velocity - state.LinearVelocity - _gravity * dt;
            acceleration = settings.FreeAcceleration;
        }
        else if (support is { } supported)
        {
            correction = Tangent(intent.Velocity - (state.LinearVelocity - supported.Velocity) - _gravity * dt, supported.Normal);
            acceleration = settings.GroundAcceleration;
        }
        else
        {
            correction = Tangent(intent.Velocity - state.LinearVelocity, up);
            acceleration = settings.AirAcceleration;
        }
        var length = correction.Length();
        var limit = acceleration * dt;
        if (!float.IsFinite(length) || !float.IsFinite(limit))
            throw new ArgumentException("Motor correction exceeds the finite solver range.");
        if (length > limit)
            correction *= limit / length;
        if (support != null && settings.StepHeight > 0)
        {
            // Check the proposed bounded movement before rejecting uphill control
            // against steep faces: a legal low stair must work even from rest.
            var travel = Tangent(state.LinearVelocity + correction - support.Value.Velocity, up) * dt;
            TryMotorStep(entry, settings, travel, up);
            state = GetState(id);
        }
        if (intent.Mode == SpatialMotorMode.Walking)
            correction = RemoveSteepUphillControl(entry, state.Pose.Compose(part.LocalPose), correction, up, settings);
        var impulse = correction * entry.Definition.Mass;
        ApplyImpulse(id, impulse, state.Pose.TransformPoint(entry.Center));
        if (support is { } reaction && _entries[reaction.Body] is { Paused: false } platform
            && platform.Definition.Motion == SpatialBackendMotion.Dynamic)
            ApplyImpulse(reaction.Body, -impulse, reaction.Point);

    }

    private void TryMotorStep(Entry entry, SpatialMotorSettings settings, Vector3 travel, Vector3 up)
    {
        if (travel.LengthSquared() < settings.ContactTolerance * settings.ContactTolerance)
            return;
        var state = GetState(entry.Id);
        var part = entry.Definition.Parts.Single();
        var pose = state.Pose.Compose(part.LocalPose);
        if (!MotorPathBlocked(entry, pose, travel, settings.ContactTolerance, MathF.Cos(settings.MaximumSlope)))
            return;
        // A capsule first meets the rounded edge before its center reaches a
        // walkable top. Search one radius ahead, checking the entire path, rather
        // than requiring this tick's short travel to clear the edge already.
        var search = travel + Vector3.Normalize(travel) * ((SpatialCapsule) part.Shape).Radius;
        var rise = up * settings.StepHeight;
        if (MotorPathBlocked(entry, pose, rise, settings.ContactTolerance))
            return;
        var raised = new RigidTransform3(pose.Position + rise, pose.Rotation);
        if (MotorPathBlocked(entry, raised, search, settings.ContactTolerance))
            return;
        var landingStart = new RigidTransform3(raised.Position + search, raised.Rotation);
        var down = -up * (settings.StepHeight + settings.SupportDistance);
        foreach (var hit in ShapeCast(part.Shape, landingStart, down, default, entry.Definition.CollisionMask, entry.Id))
        {
            if (!MotorCollides(entry, hit.Body, hit.Source))
                continue;
            // The first physical obstruction must be a walkable landing.
            if (hit.Normal is not { } normal || Vector3.Dot(normal, up) < MathF.Cos(settings.MaximumSlope))
                return;
            var height = settings.StepHeight - hit.Fraction * (settings.StepHeight + settings.SupportDistance);
            if (height <= settings.ContactTolerance || height > settings.StepHeight)
                return;
            // Lift only: horizontal travel is integrated exactly once by physics.
            SetState(entry.Id, state with { Pose = new RigidTransform3(state.Pose.Position + up * height, state.Pose.Rotation) });
            return;
        }
    }

    private bool MotorPathBlocked(Entry entry, RigidTransform3 pose, Vector3 travel, float tolerance, float? steepOnly = null)
    {
        var part = entry.Definition.Parts.Single();
        var hits = ShapeCast(part.Shape, pose, travel, default, entry.Definition.CollisionMask, entry.Id);
        var overlaps = hits.Any(h => h.Point == null)
            ? OverlapContacts(part.Shape, pose, entry.Definition.CollisionMask, entry.Id) : null;
        foreach (var hit in hits)
        {
            if (!MotorCollides(entry, hit.Body, hit.Source))
                continue;
            if (hit.Normal is { } normal)
            {
                if (steepOnly is { } threshold && Vector3.Dot(normal, pose.TransformVector(Vector3.UnitZ)) >= threshold)
                    continue;
                if (Vector3.Dot(travel, normal) < -tolerance && hit.Fraction < 1 - 1e-5f)
                    return true;
            }
            else
            {
                var manifold = overlaps!.FirstOrDefault(o => o.Body == hit.Body && o.ChildIndex == hit.ChildIndex);
                // Missing witnesses are unavailable clearance, never a free path.
                if (manifold.Contacts == null || manifold.Contacts.Count == 0)
                    return true;
                if (manifold.Contacts.Any(c => Vector3.Dot(travel, c.Normal) < -tolerance
                    && (steepOnly == null || Vector3.Dot(c.Normal, pose.TransformVector(Vector3.UnitZ)) < steepOnly.Value)))
                    return true;
            }
        }
        return false;
    }

    private Vector3 RemoveSteepUphillControl(Entry entry, RigidTransform3 pose, Vector3 correction,
        Vector3 up, SpatialMotorSettings settings)
    {
        var capsule = (SpatialCapsule) entry.Definition.Parts[0].Shape;
        // A small proximity envelope obtains normals for resting contacts whose
        // physical manifolds have tiny positive separation. It never moves a body.
        var envelope = new SpatialCapsule(capsule.Radius + settings.ContactTolerance, capsule.Length);
        foreach (var overlap in OverlapContacts(envelope, pose, entry.Definition.CollisionMask, entry.Id))
        {
            if (!MotorCollides(entry, overlap.Body, overlap.Source))
                continue;
            foreach (var contact in overlap.Contacts)
            {
                var slope = Vector3.Dot(contact.Normal, up);
                if (slope <= 1e-4f || slope >= MathF.Cos(settings.MaximumSlope))
                    continue;
                var horizontal = Vector3.Normalize(Tangent(contact.Normal, up));
                var inward = Vector3.Dot(correction, horizontal);
                if (inward < 0)
                    correction -= horizontal * inward;
            }
        }
        return correction;
    }

    internal bool IsMotorResizeClear(SpatialBackendBodyId id, SpatialBackendPart replacement)
    {
        var entry = _entries[id];
        var pose = GetState(id).Pose;
        var old = entry.Definition.Parts.Single();
        var existing = OverlapContacts(old.Shape, pose.Compose(old.LocalPose), entry.Definition.CollisionMask, id);
        var next = OverlapContacts(replacement.Shape, pose.Compose(replacement.LocalPose), entry.Definition.CollisionMask, id);
        const float epsilon = 1e-5f;
        foreach (var overlap in next)
        {
            if (!MotorCollides(entry, overlap.Body, overlap.Source))
                continue;
            var previous = existing.FirstOrDefault(o => o.Body == overlap.Body && o.ChildIndex == overlap.ChildIndex);
            foreach (var contact in overlap.Contacts)
            {
                if (contact.Depth <= epsilon)
                    continue;
                if (previous.Contacts == null || !previous.Contacts.Any(c => Vector3.Dot(c.Normal, contact.Normal) > 0.999f
                    && contact.Depth <= c.Depth + epsilon))
                    return false;
            }
        }
        return true;
    }

    private static Vector3 Tangent(Vector3 vector, Vector3 normal) => vector - normal * Vector3.Dot(vector, normal);
}
