using System.Numerics;
using NUnit.Framework;
using Robust.Shared.Maths;
using Robust.Shared.Spatial.Physics;

namespace Robust.Shared.Tests.Spatial;

[TestFixture]
public sealed class SpatialImpulseTest
{
    [Test]
    public void OffCenterImpulseUsesMassInertiaAndCanonicalOrigin()
    {
        using var backend = new BepuSpatialPhysicsBackend(Vector3.Zero);
        var body = backend.AddBody(new SpatialBackendBodyDefinition(SpatialBackendMotion.Dynamic,
            new[] { new SpatialBackendPart(new SpatialSphere(0.5f), new RigidTransform3(new Vector3(2, 0, 0), Quaternion.Identity)) },
            mass: 2), new SpatialBackendState(RigidTransform3.Identity, default, default));
        backend.ApplyImpulse(body, new Vector3(0, 2, 0), new Vector3(3, 0, 0));
        var state = backend.GetState(body);
        Assert.That(Vector3.Distance(state.AngularVelocity, new Vector3(0, 0, 10)), Is.LessThan(1e-4));
        Assert.That(Vector3.Distance(state.LinearVelocity, new Vector3(0, -19, 0)), Is.LessThan(1e-4));
        Assert.That(state.Pose, Is.EqualTo(RigidTransform3.Identity));
        var comVelocity = state.LinearVelocity + Vector3.Cross(state.AngularVelocity, new Vector3(2, 0, 0));
        Assert.That(Vector3.Distance(comVelocity, Vector3.UnitY), Is.LessThan(1e-4));
    }

    [Test]
    public void RejectedImpulseDoesNotPartiallyModifyBody()
    {
        using var backend = new BepuSpatialPhysicsBackend(Vector3.Zero);
        var parts = new[] { new SpatialBackendPart(new SpatialSphere(0.5f), RigidTransform3.Identity) };
        var initial = new SpatialBackendState(RigidTransform3.Identity, Vector3.UnitY, Vector3.UnitZ);
        var body = backend.AddBody(new SpatialBackendBodyDefinition(SpatialBackendMotion.Dynamic, parts), initial);
        Assert.Throws<ArgumentException>(() => backend.ApplyImpulse(body, new Vector3(float.NaN), Vector3.Zero));
        Assert.Throws<ArgumentException>(() => backend.ApplyImpulse(body, new Vector3(float.MaxValue), new Vector3(100)));
        Assert.That(backend.GetState(body), Is.EqualTo(initial));
        backend.ApplyImpulse(body, Vector3.Zero, Vector3.One);
        Assert.That(backend.GetState(body), Is.EqualTo(initial));
        var stationary = backend.AddBody(new SpatialBackendBodyDefinition(SpatialBackendMotion.Static, parts),
            new SpatialBackendState(new RigidTransform3(new Vector3(10), Quaternion.Identity), default, default));
        Assert.Throws<InvalidOperationException>(() => backend.ApplyImpulse(stationary, Vector3.One, Vector3.Zero));
    }

    [Test]
    public void ImpulseWakesSleepingBody()
    {
        using var backend = new BepuSpatialPhysicsBackend(Vector3.Zero);
        var body = backend.AddBody(new SpatialBackendBodyDefinition(SpatialBackendMotion.Dynamic,
            new[] { new SpatialBackendPart(new SpatialSphere(0.5f), RigidTransform3.Identity) }),
            new SpatialBackendState(RigidTransform3.Identity, default, default));
        for (var i = 0; i < 300; i++)
            backend.Step(1f / 30);
        backend.ApplyImpulse(body, Vector3.UnitX, Vector3.Zero);
        backend.Step(1f / 30);
        Assert.That(backend.GetState(body).Pose.Position.X, Is.EqualTo(1f / 30).Within(1e-5));
    }
}
