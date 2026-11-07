using System;
using System.Collections.Generic;
using System.Numerics;
using BepuPhysics;
using BepuPhysics.Collidables;
using BepuUtilities;
using Robust.Shared.Maths;

namespace Robust.Shared.Spatial.Physics;

internal sealed partial class BepuSpatialPhysicsBackend
{
    public IReadOnlyList<SpatialBackendCastHit> Overlap(SpatialConvexShape shape, RigidTransform3 pose,
        uint mask, SpatialBackendBodyId? ignoredBody = null)
        => ShapeCast(shape, pose, Vector3.Zero, Vector3.Zero, mask, ignoredBody);

    /// <summary>
    /// Cast against the current scene, with constant translation and world-axis
    /// angular displacement over [0,1]. Both broadphase levels only select
    /// candidates; BEPU convex narrowphase decides every returned hit.
    /// </summary>
    public IReadOnlyList<SpatialBackendCastHit> ShapeCast(SpatialConvexShape shape, RigidTransform3 pose,
        Vector3 translation, Vector3 angularDisplacement, uint mask, SpatialBackendBodyId? ignoredBody = null)
    {
        ThrowIfDisposed();
        SpatialBackendBodyDefinition.ValidateShape(shape);
        if (!pose.IsValid || !SpatialBackendBodyDefinition.Finite(translation)
            || !SpatialBackendBodyDefinition.Finite(angularDisplacement)
            || !float.IsFinite(angularDisplacement.LengthSquared()))
            throw new ArgumentException("A shape query requires a finite pose and displacement.");
        return shape switch
        {
            SpatialBox box => CastConvex(new Box(box.Size.X, box.Size.Y, box.Size.Z), pose,
                translation, angularDisplacement, mask, ignoredBody),
            SpatialSphere sphere => CastConvex(new Sphere(sphere.Radius), pose,
                translation, angularDisplacement, mask, ignoredBody),
            SpatialCapsule capsule => CastConvex(new Capsule(capsule.Radius, capsule.Length),
                pose.Compose(new RigidTransform3(Vector3.Zero, Quaternion.CreateFromAxisAngle(Vector3.UnitX, MathF.PI / 2))),
                translation, angularDisplacement, mask, ignoredBody),
            _ => throw new ArgumentException("Unsupported query shape.")
        };
    }

    private unsafe IReadOnlyList<SpatialBackendCastHit> CastConvex<T>(T shape, RigidTransform3 pose,
        Vector3 translation, Vector3 angularDisplacement, uint mask, SpatialBackendBodyId? ignored)
        where T : unmanaged, IConvexShape
    {
        shape.ComputeBounds(pose.Rotation, out var min, out var max);
        shape.ComputeAngularExpansionData(out var radius, out _);
        var expansion = new Vector3((float) Math.Min((double) angularDisplacement.Length() * radius, 2d * radius));
        min += pose.Position - expansion + Vector3.Min(Vector3.Zero, translation);
        max += pose.Position + expansion + Vector3.Max(Vector3.Zero, translation);
        // Reject overflow before entering BEPU's tree traversal.
        _motorCoverage?.Check(new Box3(min, max));
        _ = new Box3(min, max);
        var candidates = new CandidateBodies(this, mask, ignored, new List<Entry>());
        _simulation.BroadPhase.GetOverlaps(min, max, ref candidates);
        var hits = new List<SpatialBackendCastHit>();
        var children = new CandidateChildren(new List<int>());
        var velocity = new BodyVelocity(translation, angularDisplacement);
        foreach (var entry in candidates.Entries)
        {
            var bodyPose = entry.Definition.Motion == SpatialBackendMotion.Static
                ? _simulation.Statics.GetStaticReference(entry.Static).Pose
                : _simulation.Bodies.GetBodyReference(entry.Body).Pose;
            var inverse = Quaternion.Conjugate(bodyPose.Orientation);
            var center = Vector3.Transform((min + (max - min) * 0.5f) - bodyPose.Position, inverse);
            var half = (max - min) * 0.5f;
            var extent = Vector3.Abs(Vector3.Transform(new Vector3(half.X, 0, 0), inverse))
                + Vector3.Abs(Vector3.Transform(new Vector3(0, half.Y, 0), inverse))
                + Vector3.Abs(Vector3.Transform(new Vector3(0, 0, half.Z), inverse));
            ref var compound = ref _simulation.Shapes.GetShape<BigCompound>(entry.Shape.Index);
            children.Indices.Clear();
            compound.Tree.GetOverlaps(center - extent, center + extent, ref children);
            foreach (var index in children.Indices)
            {
                if (((entry.Definition.Parts[index].CollisionLayer ?? entry.Definition.CollisionLayer) & mask) == 0)
                    continue;
                ref var child = ref compound.Children[index];
                _simulation.Shapes[child.ShapeIndex.Type].GetShapeData(child.ShapeIndex.Index, out var data, out _);
                var targetPosition = bodyPose.Position + Vector3.Transform(child.LocalPose.Position, bodyPose.Orientation);
                var targetRotation = Quaternion.Concatenate(child.LocalPose.Orientation, bodyPose.Orientation);
                var task = _simulation.NarrowPhase.SweepTaskRegistry.GetTask(shape.TypeId, child.ShapeIndex.Type);
                if (!task.Sweep(&shape, shape.TypeId, RigidPose.Identity, pose.Rotation, velocity,
                        data, child.ShapeIndex.Type, RigidPose.Identity, targetPosition - pose.Position,
                        targetRotation, default, 1, 1e-6f, 1e-6f, 64,
                        out _, out var fraction, out var location, out var normal))
                    continue;
                // A zero-time overlap does not supply a meaningful witness in
                // BEPU's sweep API. Preserve that distinction for motor/placement.
                var initial = fraction <= 0;
                hits.Add(new SpatialBackendCastHit(entry.Id, entry.Definition.Parts[index], fraction,
                    initial ? null : pose.Position + location, initial ? null : normal, index));
            }
        }
        hits.Sort(static (a, b) => a.Fraction.CompareTo(b.Fraction));
        return hits;
    }

    private readonly struct CandidateBodies(BepuSpatialPhysicsBackend owner, uint mask,
        SpatialBackendBodyId? ignored, List<Entry> entries) : IBreakableForEach<CollidableReference>
    {
        public readonly List<Entry> Entries = entries;
        public bool LoopBody(CollidableReference reference)
        {
            var entry = owner.GetEntry(reference);
            if (entry.Id != ignored && (entry.Definition.CollisionLayerUnion & mask) != 0)
                Entries.Add(entry);
            return true;
        }
    }

    private readonly struct CandidateChildren(List<int> indices) : IBreakableForEach<int>
    {
        public readonly List<int> Indices = indices;
        public bool LoopBody(int index) { Indices.Add(index); return true; }
    }
}
