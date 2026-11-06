using System.Numerics;
using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.Constraints;
using BepuPhysics.Trees;
using BepuUtilities;
using BepuUtilities.Memory;
using NUnit.Framework;

namespace Robust.Shared.Tests.Spatial;

/// <summary>
/// E00 dependency qualification, not the E03 ECS physics backend.
/// Exercises the pinned managed dependency with SS14's +Z-up convention.
/// </summary>
[TestFixture]
public sealed class BepuQualificationTest
{
    [Test]
    public void CompoundQueryPreservesGapBetweenChildren()
    {
        var pool = new BufferPool();
        try
        {
            using var simulation = Simulation.Create(pool, new Contacts(), new Gravity(), new SolveDescription(8, 1));
            var childShape = simulation.Shapes.Add(new Box(1, 1, 1));
            pool.Take<CompoundChild>(2, out var children);
            children[0] = new CompoundChild { ShapeIndex = childShape, LocalPose = new RigidPose(new Vector3(-2, 0, 0)) };
            children[1] = new CompoundChild { ShapeIndex = childShape, LocalPose = new RigidPose(new Vector3(2, 0, 0)) };
            simulation.Statics.Add(new StaticDescription(Vector3.Zero, simulation.Shapes.Add(new Compound(children))));
            var gap = new RayHit { Distance = float.PositiveInfinity };
            simulation.RayCast(new Vector3(0, 0, 5), -Vector3.UnitZ, 10, ref gap);
            Assert.That(gap.Distance, Is.EqualTo(float.PositiveInfinity));
            var solid = new RayHit { Distance = float.PositiveInfinity };
            simulation.RayCast(new Vector3(2, 0, 5), -Vector3.UnitZ, 10, ref solid);
            Assert.That(solid.Distance, Is.EqualTo(4.5f).Within(0.001f));
        }
        finally { pool.Clear(); }
    }

    [Test]
    public void SweepReportsExistingOverlapAtZeroTime()
    {
        var pool = new BufferPool();
        try
        {
            using var simulation = Simulation.Create(pool, new Contacts(), new Gravity(), new SolveDescription(8, 1));
            simulation.Statics.Add(new StaticDescription(Vector3.Zero, simulation.Shapes.Add(new Box(1, 1, 1))));
            var hit = new SweepHit();
            simulation.Sweep(new Sphere(0.1f), new RigidPose(Vector3.Zero), new BodyVelocity(Vector3.UnitX),
                1f / 30, pool, ref hit);
            Assert.That(hit.Hit, Is.True);
            Assert.That(hit.Time, Is.Zero);
        }
        finally { pool.Clear(); }
    }

    [Test]
    public void ConstraintHoldsBodyAndReleasesWhenAnchorIsRemoved()
    {
        var pool = new BufferPool();
        try
        {
            using var simulation = Simulation.Create(pool, new Contacts(), new Gravity(), new SolveDescription(8, 1));
            var sphere = new Sphere(0.1f);
            var shape = simulation.Shapes.Add(sphere);
            var anchor = simulation.Bodies.Add(BodyDescription.CreateKinematic(new Vector3(0, 0, 3), shape, 0.01f));
            var body = simulation.Bodies.Add(BodyDescription.CreateDynamic(new Vector3(0, 0, 2),
                sphere.ComputeInertia(1), shape, 0.01f));
            simulation.Solver.Add(anchor, body, new BallSocket
            {
                LocalOffsetA = Vector3.Zero,
                LocalOffsetB = Vector3.UnitZ,
                SpringSettings = new SpringSettings(30, 1)
            });
            for (var tick = 0; tick < 120; tick++)
                simulation.Timestep(1f / 60);
            Assert.That(simulation.Bodies.GetBodyReference(body).Pose.Position.Z, Is.EqualTo(2).Within(0.03f));
            simulation.Bodies.Remove(anchor);
            for (var tick = 0; tick < 60; tick++)
                simulation.Timestep(1f / 60);
            Assert.That(simulation.Bodies.GetBodyReference(body).Pose.Position.Z, Is.LessThan(1));
        }
        finally { pool.Clear(); }
    }

    [Test]
    public void RotatingShapeSweepDetectsObstacleBetweenEndpointPoses()
    {
        var pool = new BufferPool();
        try
        {
            using var simulation = Simulation.Create(pool, new Contacts(), new Gravity(), new SolveDescription(8, 1));
            // The bar starts and ends horizontal after a half turn. A query
            // considering only those endpoint bounds would miss this obstacle.
            simulation.Statics.Add(new StaticDescription(new Vector3(0, 1.5f, 0),
                simulation.Shapes.Add(new Box(0.2f, 0.1f, 1))));
            var hit = new SweepHit();
            simulation.Sweep(new Box(4, 0.2f, 0.2f), new RigidPose(Vector3.Zero),
                new BodyVelocity(Vector3.Zero, new Vector3(0, 0, 2 * MathF.PI)), 0.5f, pool, ref hit);
            Assert.That(hit.Hit, Is.True);
            Assert.That(hit.Time, Is.GreaterThan(0).And.LessThan(0.25f));
        }
        finally { pool.Clear(); }
    }

    [TestCase(0f, true)]
    [TestCase(4f, false)]
    public void ShapeSweepRespectsHeight(float height, bool expectedHit)
    {
        var pool = new BufferPool();
        try
        {
            using var simulation = Simulation.Create(pool, new Contacts(), new Gravity(), new SolveDescription(8, 1));
            simulation.Statics.Add(new StaticDescription(Vector3.Zero,
                simulation.Shapes.Add(new Box(0.1f, 2, 2))));
            var hit = new SweepHit();
            simulation.Sweep(new Sphere(0.1f), new RigidPose(new Vector3(-2, 0, height)),
                new BodyVelocity(new Vector3(300, 0, 0)), 1f / 30, pool, ref hit);
            Assert.That(hit.Hit, Is.EqualTo(expectedHit));
            if (expectedHit)
                Assert.That(hit.Time, Is.EqualTo(1.85f / 300f).Within(0.0001f));
        }
        finally { pool.Clear(); }
    }

    private struct SweepHit : ISweepHitHandler
    {
        public bool Hit;
        public float Time;
        public bool AllowTest(CollidableReference collidable) => true;
        public bool AllowTest(CollidableReference collidable, int childIndex) => true;
        public void OnHit(ref float maximumT, float t, in Vector3 location, in Vector3 normal,
            CollidableReference collidable)
        {
            Hit = true;
            Time = maximumT = t;
        }
        public void OnHitAtZeroT(ref float maximumT, CollidableReference collidable)
        {
            Hit = true;
            Time = maximumT = 0;
        }
    }

    [Test]
    public void RayQueriesDistinguishOverlappingFloors()
    {
        var pool = new BufferPool();
        try
        {
            using var simulation = Simulation.Create(pool, new Contacts(), new Gravity(), new SolveDescription(8, 1));
            var shape = simulation.Shapes.Add(new Box(10, 10, 0.2f));
            simulation.Statics.Add(new StaticDescription(new Vector3(0, 0, -0.1f), shape));
            simulation.Statics.Add(new StaticDescription(new Vector3(0, 0, 2.9f), shape));
            foreach (var height in new[] { 1f, 4f })
            {
                var hit = new RayHit { Distance = float.PositiveInfinity };
                simulation.RayCast(new Vector3(0, 0, height), -Vector3.UnitZ, 10, ref hit);
                Assert.That(hit.Distance, Is.EqualTo(1).Within(0.001f));
                Assert.That(hit.Normal.Z, Is.EqualTo(1).Within(0.001f));
            }
        }
        finally { pool.Clear(); }
    }

    [TestCase(30)]
    [TestCase(60)]
    public void ContinuousDetectionStopsFastSphereAtThinWall(int tickRate)
    {
        var pool = new BufferPool();
        try
        {
            using var simulation = Simulation.Create(pool, new Contacts(), new Gravity(), new SolveDescription(8, 1));
            simulation.Statics.Add(new StaticDescription(Vector3.Zero,
                simulation.Shapes.Add(new Box(0.1f, 10, 10))));
            var sphere = new Sphere(0.1f);
            var body = simulation.Bodies.Add(BodyDescription.CreateDynamic(
                new Vector3(-2, 0, 0), new BodyVelocity(new Vector3(300, 0, 0)), sphere.ComputeInertia(1),
            // Default 1ms sweep tolerance spans 0.3m at this velocity,
            // exceeding the wall thickness. Keep it below the geometry scale.
                new CollidableDescription(simulation.Shapes.Add(sphere), 1f,
                    ContinuousDetection.Continuous(1e-6f, 1e-6f)), 0.01f));
            for (var tick = 0; tick < 10; tick++)
            {
                simulation.Timestep(1f / tickRate);
                var position = simulation.Bodies.GetBodyReference(body).Pose.Position;
                Assert.That(float.IsFinite(position.X) && float.IsFinite(position.Z), Is.True);
                Assert.That(position.X, Is.LessThan(-0.1f), "Body crossed a 0.1m wall at 300m/s.");
            }
        }
        finally { pool.Clear(); }
    }

    private struct RayHit : IRayHitHandler
    {
        public float Distance;
        public Vector3 Normal;
        public bool AllowTest(CollidableReference collidable) => true;
        public bool AllowTest(CollidableReference collidable, int childIndex) => true;
        public void OnRayHit(in RayData ray, ref float maximumT, float t, in Vector3 normal,
            CollidableReference collidable, int childIndex)
        {
            maximumT = t;
            Distance = t;
            Normal = normal;
        }
    }

    [TestCase(30)]
    [TestCase(60)]
    public void FallingBodyRestsOnHorizontalSupport(int tickRate)
    {
        var pool = new BufferPool();
        try
        {
            using var simulation = Simulation.Create(pool, new Contacts(), new Gravity(), new SolveDescription(8, 1));
            var sphere = new Sphere(0.5f);
            var handle = simulation.Bodies.Add(BodyDescription.CreateDynamic(
                new Vector3(0, 0, 5), sphere.ComputeInertia(1), simulation.Shapes.Add(sphere), 0.01f));
            // Top of the slab is z=0; the sphere should settle at z=0.5.
            simulation.Statics.Add(new StaticDescription(new Vector3(0, 0, -0.1f),
                simulation.Shapes.Add(new Box(20, 20, 0.2f))));
            for (var tick = 0; tick < tickRate * 5; tick++)
                simulation.Timestep(1f / tickRate);

            var body = simulation.Bodies.GetBodyReference(handle);
            Assert.That(body.Pose.Position.Z, Is.EqualTo(0.5f).Within(0.03f));
            Assert.That(body.Pose.Position.X, Is.EqualTo(0).Within(0.001f));
            Assert.That(body.Pose.Position.Y, Is.EqualTo(0).Within(0.001f));
            Assert.That(body.Velocity.Linear.Length(), Is.LessThan(0.05f));

            simulation.Bodies.Remove(handle);
            Assert.That(simulation.Bodies.BodyExists(handle), Is.False);
        }
        finally
        {
            pool.Clear();
        }
    }

    private struct Gravity : IPoseIntegratorCallbacks
    {
        private Vector3Wide _step;
        public AngularIntegrationMode AngularIntegrationMode => AngularIntegrationMode.Nonconserving;
        public bool AllowSubstepsForUnconstrainedBodies => false;
        public bool IntegrateVelocityForKinematics => false;
        public void Initialize(Simulation simulation) { }
        public void PrepareForIntegration(float dt) => _step = Vector3Wide.Broadcast(new Vector3(0, 0, -9.81f * dt));
        public void IntegrateVelocity(Vector<int> indices, Vector3Wide position, QuaternionWide orientation,
            BodyInertiaWide inertia, Vector<int> mask, int worker, Vector<float> dt, ref BodyVelocityWide velocity)
            => velocity.Linear += _step;
    }

    private struct Contacts : INarrowPhaseCallbacks
    {
        public void Initialize(Simulation simulation) { }
        public void Dispose() { }
        public bool AllowContactGeneration(int worker, CollidableReference a, CollidableReference b, ref float margin)
            => a.Mobility == CollidableMobility.Dynamic || b.Mobility == CollidableMobility.Dynamic;
        public bool AllowContactGeneration(int worker, CollidablePair pair, int a, int b) => true;
        public bool ConfigureContactManifold<T>(int worker, CollidablePair pair, ref T manifold,
            out PairMaterialProperties material) where T : unmanaged, IContactManifold<T>
        {
            material = new PairMaterialProperties
            {
                FrictionCoefficient = 1,
                MaximumRecoveryVelocity = 2,
                SpringSettings = new SpringSettings(240, 1)
            };
            return true;
        }
        public bool ConfigureContactManifold(int worker, CollidablePair pair, int a, int b,
            ref ConvexContactManifold manifold) => true;
    }
}
