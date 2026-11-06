using System.Numerics;
using NUnit.Framework;
using Robust.Shared.GameObjects;
using Robust.Shared.Maths;
using Robust.Shared.Spatial;
using Robust.Shared.Spatial.Physics;

namespace Robust.Shared.Tests.Spatial;

[TestFixture]
public sealed class SpatialPhysicsBackendTest
{
    private static SpatialBackendState State(Vector3 position, Vector3 velocity = default, Vector3 angular = default)
        => new(new RigidTransform3(position, Quaternion.Identity), velocity, angular);

    [TestCase(30)]
    [TestCase(60)]
    public void BackendSimulatesGravityAndRetainsCanonicalOrigin(int tickRate)
    {
        using ISpatialPhysicsBackend backend = new BepuSpatialPhysicsBackend(new Vector3(0, 0, -9.81f));
        backend.AddBody(Body(SpatialBackendMotion.Static, new SpatialBox(new Vector3(20, 20, 0.2f))), State(new Vector3(0, 0, -0.1f)));
        var parts = new[]
        {
            new SpatialBackendPart(new SpatialSphere(0.5f), new RigidTransform3(new Vector3(2, 0, 0), Quaternion.Identity))
        };
        var body = backend.AddBody(new SpatialBackendBodyDefinition(SpatialBackendMotion.Dynamic, parts), State(new Vector3(0, 0, 5)));
        for (var tick = 0; tick < tickRate * 5; tick++)
            backend.Step(1f / tickRate);
        var state = backend.GetState(body);
        Assert.That(state.Pose.Position.Z, Is.EqualTo(0.5).Within(0.03));
        Assert.That(state.Pose.Position.X, Is.EqualTo(0).Within(1e-4));
        Assert.That(state.LinearVelocity.Length(), Is.LessThan(0.05));
        var ray = backend.RayCast(new Vector3(2, 0, 3), -Vector3.UnitZ, 2.5f, uint.MaxValue);
        Assert.That(ray.HasValue, Is.True);
        Assert.That(ray!.Value.Body, Is.EqualTo(body));
        Assert.That(ray.Value.Distance, Is.EqualTo(2).Within(0.04));
    }

    [Test]
    public void OffCenterCompoundConvertsAngularVelocityAtEntityOrigin()
    {
        using var backend = new BepuSpatialPhysicsBackend(Vector3.Zero);
        var offset = new Vector3(2, 0, 0);
        var part = new SpatialBackendPart(new SpatialSphere(0.25f), new RigidTransform3(offset, Quaternion.Identity));
        var initial = State(Vector3.Zero, Vector3.Zero, Vector3.UnitZ);
        var body = backend.AddBody(new SpatialBackendBodyDefinition(SpatialBackendMotion.Dynamic, new[] { part }), initial);
        AssertState(backend.GetState(body), initial);
        const float dt = 0.1f;
        backend.Step(dt);
        var result = backend.GetState(body);
        // Center of mass moves at omega x offset = (0,2,0), while the
        // canonical origin rotates about that center instead of drifting by +2m.
        var center = result.Pose.TransformPoint(offset);
        Assert.That(Vector3.Distance(center, new Vector3(2, 0.2f, 0)), Is.LessThan(1e-4));
        var comVelocity = result.LinearVelocity + Vector3.Cross(result.AngularVelocity, result.Pose.TransformVector(offset));
        Assert.That(Vector3.Distance(comVelocity, new Vector3(0, 2, 0)), Is.LessThan(1e-4));

        var reset = new SpatialBackendState(new RigidTransform3(new Vector3(5, 6, 7),
            Quaternion.CreateFromAxisAngle(Vector3.UnitY, 0.7f)), new Vector3(1, 2, 3), new Vector3(-1, 0.2f, 0.5f));
        backend.SetState(body, reset);
        AssertState(backend.GetState(body), reset);
        Assert.Throws<ArgumentException>(() => backend.SetState(body,
            new SpatialBackendState(reset.Pose, Vector3.Zero, new Vector3(float.MaxValue))));
        AssertState(backend.GetState(body), reset);
    }

    [Test]
    public void KinematicVelocityIsIntegratedWithoutGravity()
    {
        using var backend = new BepuSpatialPhysicsBackend(new Vector3(0, 0, -9.81f));
        var initial = State(new Vector3(0, 0, 5), new Vector3(1, 2, 0));
        var body = backend.AddBody(Body(SpatialBackendMotion.Kinematic, new SpatialBox(Vector3.One)), initial);
        for (var tick = 0; tick < 60; tick++)
            backend.Step(1f / 60);
        AssertState(backend.GetState(body), State(new Vector3(1, 2, 5), initial.LinearVelocity));
    }

    [Test]
    public void CompoundQueriesKeepSurfaceIdentityFilteringAndHeightSeparation()
    {
        using var backend = new BepuSpatialPhysicsBackend(Vector3.Zero);
        var frame = new EntityUid(4);
        var source = new EntityUid(5);
        var parts = new[]
        {
            new SpatialBackendPart(new SpatialBox(new Vector3(4, 4, 0.2f)), RigidTransform3.Identity,
                source, new SpatialSurfaceReference(frame, new SurfaceId(7)), 11),
            new SpatialBackendPart(new SpatialBox(new Vector3(4, 4, 0.2f)),
                new RigidTransform3(new Vector3(0, 0, 3.75f), Quaternion.Identity),
                source, new SpatialSurfaceReference(frame, new SurfaceId(8)), 12)
        };
        var body = backend.AddBody(new SpatialBackendBodyDefinition(SpatialBackendMotion.Static, parts, collisionLayer: 4), State(Vector3.Zero));
        var upper = backend.RayCast(new Vector3(0, 0, 5), -Vector3.UnitZ, 10, 4)!.Value;
        Assert.That(upper.Source.Surface, Is.EqualTo(parts[1].Surface));
        Assert.That(upper.Source.Entity, Is.EqualTo(source));
        Assert.That(upper.Source.Revision, Is.EqualTo(12));
        Assert.That(upper.Distance, Is.EqualTo(1.15f).Within(1e-5));
        Assert.That(upper.Normal, Is.EqualTo(Vector3.UnitZ));
        var lower = backend.RayCast(new Vector3(0, 0, 2), -Vector3.UnitZ, 10, 4)!.Value;
        Assert.That(lower.Source.Surface, Is.EqualTo(parts[0].Surface));
        Assert.That(lower.Distance, Is.EqualTo(1.9f).Within(1e-5));
        Assert.That(backend.RayCast(new Vector3(0, 0, 5), -Vector3.UnitZ, 10, 2), Is.Null);
        Assert.That(backend.RayCast(new Vector3(0, 0, 5), -Vector3.UnitZ, 10, 4, body), Is.Null);
        backend.RemoveBody(body);
        Assert.That(backend.RayCast(new Vector3(0, 0, 5), -Vector3.UnitZ, 10, 4), Is.Null);
    }

    [Test]
    public void CapsulesAreZUpAndMovingStaticsUpdateQueriesImmediately()
    {
        using var backend = new BepuSpatialPhysicsBackend(Vector3.Zero);
        var body = backend.AddBody(Body(SpatialBackendMotion.Static, new SpatialCapsule(0.25f, 1.5f)), State(Vector3.Zero));
        Assert.That(backend.RayCast(new Vector3(0, 0, 2), -Vector3.UnitZ, 3, 1)!.Value.Distance, Is.EqualTo(1).Within(1e-5));
        Assert.That(backend.RayCast(new Vector3(0, 2, 0), -Vector3.UnitY, 3, 1)!.Value.Distance, Is.EqualTo(1.75).Within(1e-5));
        backend.SetState(body, State(new Vector3(10, 0, 0)));
        Assert.That(backend.RayCast(new Vector3(0, 0, 2), -Vector3.UnitZ, 3, 1), Is.Null);
        Assert.That(backend.RayCast(new Vector3(10, 0, 2), -Vector3.UnitZ, 3, 1), Is.Not.Null);
    }

    [TestCase(0.02f, 0.05f, 30)]
    [TestCase(0.02f, 0.05f, 60)]
    [TestCase(0.1f, 0.1f, 30)]
    [TestCase(0.1f, 0.1f, 60)]
    public void ContinuousBodyDoesNotTunnelAndCollisionMasksApplyToBothParticipants(float thickness, float radius, int tickRate)
    {
        using var backend = new BepuSpatialPhysicsBackend(Vector3.Zero);
        backend.AddBody(new SpatialBackendBodyDefinition(SpatialBackendMotion.Static,
            new[] { new SpatialBackendPart(new SpatialBox(new Vector3(thickness, 10, 10)), RigidTransform3.Identity) },
            collisionLayer: 2, collisionMask: 1), State(Vector3.Zero));
        var blocked = backend.AddBody(Body(SpatialBackendMotion.Dynamic, new SpatialSphere(radius)), State(new Vector3(-2, 0, 0), new Vector3(300, 0, 0)));
        var ignored = backend.AddBody(new SpatialBackendBodyDefinition(SpatialBackendMotion.Dynamic,
            new[] { new SpatialBackendPart(new SpatialSphere(0.05f), RigidTransform3.Identity) }, collisionLayer: 4),
            State(new Vector3(-2, 2, 0), new Vector3(300, 0, 0)));
        backend.Step(1f / tickRate);
        Assert.That(backend.GetState(blocked).Pose.Position.X, Is.LessThanOrEqualTo(-0.04));
        Assert.That(backend.GetState(ignored).Pose.Position.X, Is.EqualTo(-2 + 300f / tickRate).Within(1e-4));
    }

    [TestCase(30)]
    [TestCase(60)]
    public void RotatingCompoundSweepsIncludeOffsetChildren(int tickRate)
    {
        using var backend = new BepuSpatialPhysicsBackend(Vector3.Zero);
        backend.AddBody(Body(SpatialBackendMotion.Static, new SpatialBox(new Vector3(100, 0.1f, 100))),
            State(new Vector3(0, 1.5f, 0)));
        var parts = new[]
        {
            new SpatialBackendPart(new SpatialBox(new Vector3(2, 0.2f, 0.2f)),
                new RigidTransform3(-Vector3.UnitX, Quaternion.Identity)),
            new SpatialBackendPart(new SpatialBox(new Vector3(2, 0.2f, 0.2f)),
                new RigidTransform3(Vector3.UnitX, Quaternion.Identity))
        };
        var body = backend.AddBody(new SpatialBackendBodyDefinition(SpatialBackendMotion.Dynamic, parts),
            State(Vector3.Zero, angular: new Vector3(0, 0, 60)));
        for (var tick = 0; tick < 10; tick++)
        {
            backend.Step(1f / tickRate);
            var pose = backend.GetState(body).Pose;
            var extent = 2 * MathF.Abs(pose.TransformVector(Vector3.UnitX).Y)
                + 0.1f * MathF.Abs(pose.TransformVector(Vector3.UnitY).Y)
                + 0.1f * MathF.Abs(pose.TransformVector(Vector3.UnitZ).Y);
            Assert.That(pose.Position.Y + extent, Is.LessThanOrEqualTo(1.48f));
        }
    }

    [Test]
    public void InvalidStateIsAtomicAndRemovedHandlesCannotAliasNewBodies()
    {
        var backend = new BepuSpatialPhysicsBackend(Vector3.Zero);
        var definition = Body(SpatialBackendMotion.Static, new SpatialSphere(0.5f));
        var first = backend.AddBody(definition, State(Vector3.Zero));
        Assert.Throws<ArgumentException>(() => backend.SetState(first, State(Vector3.Zero, Vector3.UnitX)));
        Assert.Throws<ArgumentException>(() => backend.SetState(first, new SpatialBackendState(default, default, default)));
        AssertState(backend.GetState(first), State(Vector3.Zero));
        Assert.Throws<ArgumentException>(() => backend.AddBody(definition, new SpatialBackendState(default, default, default)));
        Assert.That(backend.BodyCount, Is.EqualTo(1));
        var invalidInertia = new SpatialBackendBodyDefinition(SpatialBackendMotion.Dynamic, new[]
        {
            new SpatialBackendPart(new SpatialBox(Vector3.One), RigidTransform3.Identity),
            new SpatialBackendPart(new SpatialSphere(1e-30f), RigidTransform3.Identity)
        });
        Assert.Throws<ArgumentException>(() => backend.AddBody(invalidInertia, State(Vector3.Zero)));
        Assert.That(backend.BodyCount, Is.EqualTo(1));
        for (var i = 0; i < 100; i++)
        {
            var id = backend.AddBody(definition, State(new Vector3(i + 2, 0, 0)));
            backend.RemoveBody(id);
            Assert.Throws<KeyNotFoundException>(() => backend.GetState(id));
        }
        backend.RemoveBody(first);
        Assert.That(backend.BodyCount, Is.Zero);
        backend.Dispose();
        backend.Dispose();
        Assert.Throws<ObjectDisposedException>(() => backend.Step(1f / 30));
        Assert.Throws<ObjectDisposedException>(() => backend.AddBody(definition, State(Vector3.Zero)));
    }

    private static SpatialBackendBodyDefinition Body(SpatialBackendMotion motion, SpatialConvexShape shape)
        => new(motion, new[] { new SpatialBackendPart(shape, RigidTransform3.Identity) });

    private static void AssertState(SpatialBackendState actual, SpatialBackendState expected)
    {
        Assert.That(Vector3.Distance(actual.Pose.Position, expected.Pose.Position), Is.LessThan(1e-5));
        Assert.That(MathF.Abs(Quaternion.Dot(actual.Pose.Rotation, expected.Pose.Rotation)), Is.EqualTo(1).Within(1e-5));
        Assert.That(Vector3.Distance(actual.LinearVelocity, expected.LinearVelocity), Is.LessThan(1e-5));
        Assert.That(Vector3.Distance(actual.AngularVelocity, expected.AngularVelocity), Is.LessThan(1e-5));
    }
}
