// Based on BEPUphysics 2.4.0 compound sweep overlap finders.
// Copyright Bepu Entertainment LLC. Licensed under Apache-2.0.
// Modified: traverse the already time-scaled displacement over [0, 1].
using System.Numerics;
using System.Runtime.CompilerServices;
using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.CollisionDetection.CollisionTasks;
using BepuPhysics.CollisionDetection.SweepTasks;
using BepuUtilities.Memory;

namespace Robust.Shared.Spatial.Physics;

/// <summary>
/// BEPU 2.4.0's bounds helper returns displacement (relative velocity * maximumT).
/// Its default compound finders then traverse that displacement for maximumT
/// again. Register corrected finders without altering the pinned dependency.
/// Angular expansion still uses the actual maximumT in the bounds helper.
/// </summary>
internal static class SpatialSweepRegistration
{
    public static void Register(SweepTaskRegistry registry)
    {
        registry.Register(new CompoundPairSweepTask<Compound, Compound, CompoundFinder<Compound, Compound>>());
        registry.Register(new CompoundPairSweepTask<Compound, BigCompound, CompoundFinder<Compound, BigCompound>>());
        registry.Register(new CompoundPairSweepTask<BigCompound, BigCompound, CompoundFinder<BigCompound, BigCompound>>());
        RegisterConvex<Sphere, SphereWide>();
        RegisterConvex<Box, BoxWide>();
        RegisterConvex<Capsule, CapsuleWide>();

        void RegisterConvex<T, TWide>() where T : unmanaged, IConvexShape where TWide : unmanaged, IShapeWide<T>
        {
            registry.Register(new ConvexCompoundSweepTask<T, TWide, Compound, ConvexFinder<T, Compound>>());
            registry.Register(new ConvexCompoundSweepTask<T, TWide, BigCompound, ConvexFinder<T, BigCompound>>());
        }
    }

    private struct CompoundFinder<TA, TB> : ICompoundPairSweepOverlapFinder<TA, TB>
        where TA : unmanaged, ICompoundShape where TB : unmanaged, ICompoundShape
    {
        public unsafe void FindOverlaps(ref TA a, in Quaternion orientationA, in BodyVelocity velocityA,
            ref TB b, in Vector3 offsetB, in Quaternion orientationB, in BodyVelocity velocityB, float maximumT,
            Shapes shapes, BufferPool pool, out CompoundPairSweepOverlaps overlaps)
        {
            overlaps = new CompoundPairSweepOverlaps(pool, a.ChildCount);
            for (var i = 0; i < a.ChildCount; i++)
            {
                ref var child = ref a.GetChild(i);
                BoundingBoxHelpers.GetLocalBoundingBoxForSweep(child.ShapeIndex, shapes, child.LocalPose,
                    orientationA, velocityA, offsetB, orientationB, velocityB, maximumT,
                    out var displacement, out var min, out var max);
                ref var childOverlaps = ref overlaps.GetOverlapsForChild(i);
                childOverlaps.ChildIndex = i;
                b.FindLocalOverlaps<ChildOverlapsCollection>(min, max, displacement, 1f, pool, shapes,
                    Unsafe.AsPointer(ref childOverlaps));
            }
        }
    }

    private struct ConvexFinder<T, TB> : IConvexCompoundSweepOverlapFinder<T, TB>
        where T : unmanaged, IConvexShape where TB : unmanaged, ICompoundShape
    {
        public unsafe void FindOverlaps(ref T a, in Quaternion orientationA, in BodyVelocity velocityA,
            ref TB b, in Vector3 offsetB, in Quaternion orientationB, in BodyVelocity velocityB, float maximumT,
            Shapes shapes, BufferPool pool, out ChildOverlapsCollection overlaps)
        {
            BoundingBoxHelpers.GetLocalBoundingBoxForSweep(ref a, orientationA, velocityA,
                offsetB, orientationB, velocityB, maximumT, out var displacement, out var min, out var max);
            overlaps = default;
            b.FindLocalOverlaps<ChildOverlapsCollection>(min, max, displacement, 1f, pool, shapes,
                Unsafe.AsPointer(ref overlaps));
        }
    }
}
