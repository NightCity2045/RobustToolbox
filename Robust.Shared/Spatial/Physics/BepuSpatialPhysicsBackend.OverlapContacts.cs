using System;
using System.Collections.Generic;
using System.Numerics;
using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using Robust.Shared.Maths;

namespace Robust.Shared.Spatial.Physics;

internal sealed partial class BepuSpatialPhysicsBackend
{
    /// <summary>Actual convex manifolds, without inserting a temporary body or changing the scene.</summary>
    public IReadOnlyList<SpatialBackendOverlap> OverlapContacts(SpatialConvexShape shape, RigidTransform3 pose,
        uint mask, SpatialBackendBodyId? ignoredBody = null)
    {
        var overlaps = Overlap(shape, pose, mask, ignoredBody);
        if (overlaps.Count == 0)
            return Array.Empty<SpatialBackendOverlap>();
        return shape switch
        {
            SpatialBox box => CollectOverlapContacts(new Box(box.Size.X, box.Size.Y, box.Size.Z), pose, overlaps),
            SpatialSphere sphere => CollectOverlapContacts(new Sphere(sphere.Radius), pose, overlaps),
            SpatialCapsule capsule => CollectOverlapContacts(new Capsule(capsule.Radius, capsule.Length),
                pose.Compose(new RigidTransform3(Vector3.Zero, Quaternion.CreateFromAxisAngle(Vector3.UnitX, MathF.PI / 2))), overlaps),
            _ => throw new ArgumentException("Unsupported query shape.")
        };
    }

    private unsafe IReadOnlyList<SpatialBackendOverlap> CollectOverlapContacts<T>(T shape, RigidTransform3 pose,
        IReadOnlyList<SpatialBackendCastHit> overlaps) where T : unmanaged, IConvexShape
    {
        var result = new List<SpatialBackendOverlap>();
        var callbacks = new OverlapCallbacks(pose.Position, overlaps, result);
        var batcher = new CollisionBatcher<OverlapCallbacks>(_pool, _simulation.Shapes,
            _simulation.NarrowPhase.CollisionTaskRegistry, 0, callbacks);
        try
        {
            for (var i = 0; i < overlaps.Count; i++)
            {
                var hit = overlaps[i];
                var entry = _entries[hit.Body];
                var bodyPose = SolverPose(entry);
                ref var compound = ref _simulation.Shapes.GetShape<BigCompound>(entry.Shape.Index);
                ref var child = ref compound.Children[hit.ChildIndex];
                _simulation.Shapes[child.ShapeIndex.Type].GetShapeData(child.ShapeIndex.Index, out var data, out _);
                var position = bodyPose.Position + Vector3.Transform(child.LocalPose.Position, bodyPose.Orientation);
                var rotation = Quaternion.Concatenate(child.LocalPose.Orientation, bodyPose.Orientation);
                batcher.AddDirectly(shape.TypeId, child.ShapeIndex.Type, &shape, data,
                    position - pose.Position, pose.Rotation, rotation, 0, new PairContinuation(i));
            }
        }
        finally
        {
            // Flush also releases the batcher's private buffers. No user callback
            // runs inside it; witness conversion is detached from entity systems.
            batcher.Flush();
        }
        return result.AsReadOnly();
    }

    private readonly struct OverlapCallbacks(Vector3 origin, IReadOnlyList<SpatialBackendCastHit> pairs,
        List<SpatialBackendOverlap> result) : ICollisionCallbacks
    {
        public void OnPairCompleted<T>(int pairId, ref T manifold) where T : unmanaged, IContactManifold<T>
        {
            var contacts = new List<SpatialContactGeometry>();
            for (var i = 0; i < manifold.Count; i++)
            {
                manifold.GetContact(i, out var offset, out var normal, out var depth, out _);
                if (depth >= 0)
                    contacts.Add(new SpatialContactGeometry(origin + offset, normal, depth));
            }
            if (contacts.Count == 0)
                return;
            var pair = pairs[pairId];
            result.Add(new SpatialBackendOverlap(pair.Body, pair.Source, contacts.AsReadOnly()));
        }
        public void OnChildPairCompleted(int pairId, int childA, int childB, ref ConvexContactManifold manifold) { }
        public bool AllowCollisionTesting(int pairId, int childA, int childB) => true;
    }
}
