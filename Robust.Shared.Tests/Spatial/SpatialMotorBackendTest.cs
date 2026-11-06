using System.Numerics;
using NUnit.Framework;
using Robust.Shared.Maths;
using Robust.Shared.Spatial.Physics;

namespace Robust.Shared.Tests.Spatial;

[TestFixture]
public sealed class SpatialMotorBackendTest
{
    private static readonly SpatialMotorSettings Settings = SpatialMotorSettings.Default;
    private static readonly SpatialBackendMaterial Material = SpatialBackendMaterial.Default with { Friction = 0 };

    private static SpatialBackendBodyId Box(BepuSpatialPhysicsBackend backend, Vector3 size, Vector3 center,
        SpatialBackendMotion motion = SpatialBackendMotion.Static, Vector3 velocity = default, Quaternion? rotation = null,
        uint collisionMask = uint.MaxValue)
        => backend.AddBody(new SpatialBackendBodyDefinition(motion,
            new[] { new SpatialBackendPart(new SpatialBox(size), RigidTransform3.Identity) },
            mass: 1000, material: Material, collisionMask: collisionMask),
            new SpatialBackendState(new RigidTransform3(center, rotation ?? Quaternion.Identity), velocity, default));

    private static SpatialBackendBodyId Motor(BepuSpatialPhysicsBackend backend, Vector3 center)
        => backend.AddBody(new SpatialBackendBodyDefinition(SpatialBackendMotion.Dynamic,
            new[] { new SpatialBackendPart(new SpatialCapsule(0.3f, 1), RigidTransform3.Identity) },
            mass: 80, material: Material, lockRotation: true),
            new SpatialBackendState(new RigidTransform3(center, Quaternion.Identity), default, default));

    private static void Run(BepuSpatialPhysicsBackend backend, SpatialBackendBodyId motor, Vector3 velocity,
        int ticks, SpatialMotorSettings? settings = null, SpatialMotorMode mode = SpatialMotorMode.Walking, int hz = 60)
    {
        var config = settings ?? Settings;
        var supported = backend.FindMotorSupport(motor, config) is { Distance: <= 0.002f };
        for (var i = 0; i < ticks; i++)
        {
            backend.PrepareMotorStep(motor, config, new SpatialMotorIntent(velocity, mode), supported, 1f / hz);
            backend.Step(1f / hz);
            supported = backend.FindMotorSupport(motor, config) is { Distance: <= 0.002f };
        }
    }

    [TestCase(30)]
    [TestCase(60)]
    public void WalkStopsAtThinWallAndSlidesWithoutTilting(int hz)
    {
        using var backend = new BepuSpatialPhysicsBackend(-Vector3.UnitZ * 9.81f);
        Box(backend, new Vector3(100, 100, 1), -Vector3.UnitZ * 0.5f);
        Box(backend, new Vector3(0.02f, 100, 10), new Vector3(2, 0, 5));
        var motor = Motor(backend, Vector3.UnitZ * 0.8f);
        Run(backend, motor, new Vector3(30, 2, 0), hz * 2, hz: hz);
        var state = backend.GetState(motor);
        Assert.That(state.Pose.Position.X, Is.LessThan(1.72));
        Assert.That(state.Pose.Position.Y, Is.GreaterThan(1));
        Assert.That(state.Pose.Position.Z, Is.EqualTo(0.8).Within(0.02));
        Assert.That(state.Pose.Rotation, Is.EqualTo(Quaternion.Identity));
        Assert.That(state.AngularVelocity, Is.EqualTo(Vector3.Zero));
    }

    [TestCase(0)]
    [TestCase(0.7042f)]
    public void StepSucceedsOnlyWithWalkableTopAndHeadClearance(float startX)
    {
        foreach (var ceiling in new[] { false, true })
        {
            using var backend = new BepuSpatialPhysicsBackend(-Vector3.UnitZ * 9.81f);
            Box(backend, new Vector3(30, 10, 1), -Vector3.UnitZ * 0.5f);
            Box(backend, new Vector3(6, 10, 0.25f), new Vector3(4, 0, 0.125f));
            if (ceiling)
                Box(backend, new Vector3(10, 10, 1), new Vector3(3, 0, 2.15f)); // 5 cm above standing head.
            var motor = Motor(backend, new Vector3(startX, 0, 0.8f));
            Run(backend, motor, Vector3.UnitX * 2, 150);
            var state = backend.GetState(motor);
            if (ceiling)
                Assert.That(state.Pose.Position.X, Is.LessThan(1.01));
            else
            {
                Assert.That(state.Pose.Position.X, Is.GreaterThan(2));
                Assert.That(state.Pose.Position.Z, Is.EqualTo(1.05).Within(0.02));
            }
        }
    }

    [Test]
    public void SteepObstacleIsNotAStairAndRemovedSupportFalls()
    {
        using var backend = new BepuSpatialPhysicsBackend(-Vector3.UnitZ * 9.81f);
        var floor = Box(backend, new Vector3(30, 10, 1), -Vector3.UnitZ * 0.5f);
        Box(backend, new Vector3(6, 10, 1), new Vector3(4, 0, 0.5f));
        var motor = Motor(backend, Vector3.UnitZ * 0.8f);
        Run(backend, motor, Vector3.UnitX * 2, 150);
        Assert.That(backend.GetState(motor).Pose.Position.X, Is.LessThan(0.72));
        backend.RemoveBody(floor);
        Run(backend, motor, Vector3.Zero, 60);
        Assert.That(backend.FindMotorSupport(motor, Settings), Is.Null);
        Assert.That(backend.GetState(motor).Pose.Position.Z, Is.LessThan(-2));
    }

    [Test]
    public void MovingPlatformVelocityIsAddedOnceAndJumpSeparates()
    {
        using var backend = new BepuSpatialPhysicsBackend(-Vector3.UnitZ * 9.81f);
        var floor = Box(backend, new Vector3(30, 30, 1), -Vector3.UnitZ * 0.5f,
            SpatialBackendMotion.Kinematic, Vector3.UnitX);
        var motor = Motor(backend, Vector3.UnitZ * 0.8f);
        Run(backend, motor, Vector3.UnitX, 120);
        Assert.That(backend.GetState(motor).LinearVelocity.X, Is.EqualTo(2).Within(0.02));
        Assert.That(backend.GetState(motor).Pose.Position.X, Is.InRange(3.8f, 4.1f));
        var support = backend.FindMotorSupport(motor, Settings)!.Value;
        Assert.That(support.Body, Is.EqualTo(floor));
        Assert.That(support.Velocity, Is.EqualTo(Vector3.UnitX));
        backend.ApplyImpulse(motor, Vector3.UnitZ * 400, backend.GetState(motor).Pose.Position);
        Run(backend, motor, Vector3.Zero, 10);
        Assert.That(backend.GetState(motor).Pose.Position.Z, Is.GreaterThan(1.3));
        Assert.That(backend.FindMotorSupport(motor, Settings), Is.Null);
    }

    [Test]
    public void ResizeRetainsFootAndRejectsNewCeilingPenetration()
    {
        using var backend = new BepuSpatialPhysicsBackend(Vector3.Zero);
        Box(backend, new Vector3(10, 10, 1), -Vector3.UnitZ * 0.5f);
        Box(backend, new Vector3(10, 10, 1), Vector3.UnitZ * 2.2f);
        var motor = Motor(backend, Vector3.UnitZ * 0.8f);
        Assert.That(backend.IsMotorResizeClear(motor, new SpatialBackendPart(new SpatialCapsule(0.3f, 0.5f),
            new RigidTransform3(-Vector3.UnitZ * 0.25f, Quaternion.Identity))), Is.True);
        Assert.That(backend.IsMotorResizeClear(motor, new SpatialBackendPart(new SpatialCapsule(0.3f, 1.5f),
            new RigidTransform3(Vector3.UnitZ * 0.25f, Quaternion.Identity))), Is.False);
        Assert.That(backend.GetState(motor).Pose.Position.Z, Is.EqualTo(0.8f));
    }

    [Test]
    public void FreeControlRespectsAccelerationAndCollisionMasks()
    {
        using var backend = new BepuSpatialPhysicsBackend(Vector3.Zero);
        Box(backend, new Vector3(100, 100, 0.1f), Vector3.UnitZ * 2, collisionMask: 0);
        var motor = Motor(backend, Vector3.Zero);
        Run(backend, motor, Vector3.UnitZ * 5, 60, mode: SpatialMotorMode.Free);
        Assert.That(backend.GetState(motor).Pose.Position.Z, Is.GreaterThan(4));
        Assert.That(backend.GetState(motor).LinearVelocity.Z, Is.EqualTo(5).Within(0.001));
        Assert.That(backend.FindMotorSupport(motor, Settings), Is.Null);
    }

    [TestCase(25, true)]
    [TestCase(60, false)]
    public void SlopeLimitAllowsRampsAndSlidesOnSteepFaces(int degrees, bool walkable)
    {
        using var backend = new BepuSpatialPhysicsBackend(-Vector3.UnitZ * 9.81f);
        var angle = degrees * MathF.PI / 180;
        Box(backend, new Vector3(40, 20, 1), Vector3.Zero,
            rotation: Quaternion.CreateFromAxisAngle(Vector3.UnitY, -angle));
        var motor = Motor(backend, Vector3.UnitZ * (0.5f + 0.8f / MathF.Cos(angle)));
        Run(backend, motor, Vector3.UnitX * 2, 180, Settings with { AirAcceleration = 100 });
        var state = backend.GetState(motor);
        if (walkable)
        {
            Assert.That(state.Pose.Position.X, Is.GreaterThan(3));
            Assert.That(backend.FindMotorSupport(motor, Settings), Is.Not.Null);
            Assert.That(state.Pose.Position.Z,
                Is.EqualTo(0.5f + (0.8f + MathF.Sin(angle) * state.Pose.Position.X) / MathF.Cos(angle)).Within(0.03));
        }
        else
        {
            Assert.That(backend.FindMotorSupport(motor, Settings), Is.Null);
            Assert.That(state.Pose.Position.X, Is.LessThan(0));
        }
    }

    [Test]
    public void RotatingSupportUsesVelocityAtContactRatherThanOrigin()
    {
        using var backend = new BepuSpatialPhysicsBackend(-Vector3.UnitZ * 9.81f);
        var platform = Box(backend, new Vector3(20, 20, 1), -Vector3.UnitZ * 0.5f, SpatialBackendMotion.Kinematic);
        var platformState = backend.GetState(platform);
        backend.SetState(platform, platformState with { AngularVelocity = Vector3.UnitZ });
        var motor = Motor(backend, new Vector3(2, 0, 0.8f));
        var support = backend.FindMotorSupport(motor, Settings)!.Value;
        Assert.That(support.Velocity.Y, Is.EqualTo(2).Within(0.01));
        Run(backend, motor, Vector3.Zero, 60);
        var state = backend.GetState(motor);
        Assert.That(state.Pose.Position.X, Is.EqualTo(2 * MathF.Cos(1)).Within(0.15));
        Assert.That(state.Pose.Position.Y, Is.EqualTo(2 * MathF.Sin(1)).Within(0.15));
        Assert.That(state.Pose.Rotation, Is.EqualTo(Quaternion.Identity));
    }

    [Test]
    public void WalkingTransfersEqualAndOppositeImpulseToDynamicSupport()
    {
        using var backend = new BepuSpatialPhysicsBackend(Vector3.Zero);
        var platform = Box(backend, new Vector3(20, 20, 1), -Vector3.UnitZ * 0.5f, SpatialBackendMotion.Dynamic);
        var motor = Motor(backend, Vector3.UnitZ * 0.8f);
        backend.PrepareMotorStep(motor, Settings, new SpatialMotorIntent(Vector3.UnitX * 2, SpatialMotorMode.Walking), true, 1f / 60);
        var a = backend.GetState(platform);
        var b = backend.GetState(motor);
        Assert.That(a.LinearVelocity.X, Is.LessThan(0));
        Assert.That(b.LinearVelocity.X, Is.GreaterThan(0));
        Assert.That((a.LinearVelocity * 1000 + b.LinearVelocity * 80).Length(), Is.LessThan(0.001));
    }
}
