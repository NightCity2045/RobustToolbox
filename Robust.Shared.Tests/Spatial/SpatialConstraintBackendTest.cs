using System.Numerics;
using NUnit.Framework;
using Robust.Shared.Maths;
using Robust.Shared.Spatial.Physics;

namespace Robust.Shared.Tests.Spatial;

[TestFixture]
public sealed class SpatialConstraintBackendTest
{
    private static SpatialBackendBodyId Body(BepuSpatialPhysicsBackend backend, SpatialBackendMotion motion,
        Vector3 origin, Vector3 offset = default, Quaternion? rotation = null)
        => backend.AddBody(new SpatialBackendBodyDefinition(motion,
                new[] { new SpatialBackendPart(new SpatialSphere(0.25f), new RigidTransform3(offset, Quaternion.Identity)) }),
            new SpatialBackendState(new RigidTransform3(origin, rotation ?? Quaternion.Identity), default, default));

    private static void Steps(BepuSpatialPhysicsBackend backend, int count = 150)
    {
        for (var i = 0; i < count; i++)
            backend.Step(1f / 60);
    }

    [Test]
    public void FixedConstraintUsesCanonicalOriginsAndFollowsMovedStaticAnchor()
    {
        using var backend = new BepuSpatialPhysicsBackend(new Vector3(0, 0, -9.81f));
        var rotation = Quaternion.CreateFromAxisAngle(Vector3.UnitY, MathF.PI / 2);
        var a = Body(backend, SpatialBackendMotion.Static, Vector3.Zero);
        var b = Body(backend, SpatialBackendMotion.Dynamic, new Vector3(2, 0, 0), Vector3.UnitX, rotation);
        var relative = new RigidTransform3(new Vector3(2, 0, 0), rotation);
        var constraint = backend.AddConstraint(new SpatialBackendConstraintDefinition(SpatialConstraintKind.Fixed,
            a, b, default, default, relative));
        backend.ApplyImpulse(b, Vector3.UnitY, new Vector3(2, 0, 0));
        Steps(backend, 300);
        var pose = backend.GetState(b).Pose;
        Assert.That(Vector3.Distance(pose.Position, relative.Position), Is.LessThan(0.005));
        Assert.That(MathF.Abs(Quaternion.Dot(pose.Rotation, rotation)), Is.GreaterThan(0.9999));
        backend.SetState(a, new SpatialBackendState(new RigidTransform3(new Vector3(10, 0, 0), Quaternion.Identity), default, default));
        Steps(backend);
        Assert.That(Vector3.Distance(backend.GetState(b).Pose.Position, new Vector3(12, 0, 0)), Is.LessThan(0.005));
        Assert.That(backend.BodyCount, Is.EqualTo(2));
        backend.RemoveConstraint(constraint);
        Steps(backend, 60);
        Assert.That(backend.GetState(b).Pose.Position.Z, Is.LessThan(-4));
        Assert.That(backend.ConstraintCount, Is.Zero);
    }

    [Test]
    public void PointConstraintAllowsRotationWhileKeepingAttachmentTogether()
    {
        using var backend = new BepuSpatialPhysicsBackend(new Vector3(0, 0, -9.81f));
        var a = Body(backend, SpatialBackendMotion.Static, Vector3.Zero);
        var b = Body(backend, SpatialBackendMotion.Dynamic, -Vector3.UnitZ);
        backend.AddConstraint(new SpatialBackendConstraintDefinition(SpatialConstraintKind.Point,
            a, b, default, Vector3.UnitZ, RigidTransform3.Identity));
        backend.ApplyImpulse(b, Vector3.UnitX, -Vector3.UnitZ);
        Steps(backend, 30);
        var state = backend.GetState(b);
        Assert.That(state.Pose.TransformPoint(Vector3.UnitZ).Length(), Is.LessThan(0.01));
        Assert.That(state.Pose.Position.X, Is.GreaterThan(0.01));
        backend.RemoveBody(a);
        Assert.That(backend.ConstraintCount, Is.Zero);
        Steps(backend, 60);
        Assert.That(backend.GetState(b).Pose.Position.Z, Is.LessThan(-4));
    }

    [Test]
    public void DistanceRangeEnforcesBothLimitsWithoutTurningIntoFixedJoint()
    {
        using var backend = new BepuSpatialPhysicsBackend(Vector3.Zero);
        var a = Body(backend, SpatialBackendMotion.Static, Vector3.Zero);
        var b = Body(backend, SpatialBackendMotion.Dynamic, new Vector3(3, 0, 0));
        backend.AddConstraint(new SpatialBackendConstraintDefinition(SpatialConstraintKind.DistanceRange,
            a, b, default, default, RigidTransform3.Identity, 1, 2));
        Steps(backend);
        Assert.That(backend.GetState(b).Pose.Position.Length(), Is.InRange(0.99f, 2.01f));
        backend.SetState(b, new SpatialBackendState(new RigidTransform3(new Vector3(1.5f, 0, 0), Quaternion.Identity), default, default));
        Steps(backend);
        Assert.That(backend.GetState(b).Pose.Position.X, Is.EqualTo(1.5f).Within(1e-5));
        backend.SetState(b, new SpatialBackendState(new RigidTransform3(new Vector3(0.5f, 0, 0), Quaternion.Identity), default, default));
        Steps(backend);
        Assert.That(backend.GetState(b).Pose.Position.Length(), Is.InRange(0.99f, 2.01f));
    }

    [Test]
    public void LastConstraintRemovalRestoresContactsAndOldIdsCannotAliasNewLinks()
    {
        using var backend = new BepuSpatialPhysicsBackend(Vector3.Zero);
        var a = Body(backend, SpatialBackendMotion.Dynamic, Vector3.Zero);
        var b = Body(backend, SpatialBackendMotion.Dynamic, Vector3.UnitX * 0.1f);
        var definition = new SpatialBackendConstraintDefinition(SpatialConstraintKind.Fixed, a, b, default, default,
            new RigidTransform3(Vector3.UnitX * 0.1f, Quaternion.Identity));
        var first = backend.AddConstraint(definition);
        var second = backend.AddConstraint(definition);
        Steps(backend, 300);
        backend.RemoveConstraint(first);
        Steps(backend, 60);
        Assert.That(Vector3.Distance(backend.GetState(a).Pose.Position, backend.GetState(b).Pose.Position), Is.EqualTo(0.1f).Within(0.001));
        backend.RemoveConstraint(second);
        Steps(backend, 60);
        Assert.That(Vector3.Distance(backend.GetState(a).Pose.Position, backend.GetState(b).Pose.Position), Is.GreaterThan(0.49));
        var next = backend.AddConstraint(definition);
        Assert.That(next, Is.Not.EqualTo(first));
        Assert.Throws<KeyNotFoundException>(() => backend.RemoveConstraint(first));
        backend.RemoveBody(b);
        Assert.That(backend.ConstraintCount, Is.Zero);
    }

    [Test]
    public void InvalidConstraintDoesNotCreateAnchorsOrDisableContacts()
    {
        using var backend = new BepuSpatialPhysicsBackend(Vector3.Zero);
        var a = Body(backend, SpatialBackendMotion.Static, Vector3.Zero);
        var b = Body(backend, SpatialBackendMotion.Static, Vector3.UnitX);
        var definition = new SpatialBackendConstraintDefinition(SpatialConstraintKind.Point,
            a, b, default, default, RigidTransform3.Identity);
        Assert.Throws<InvalidOperationException>(() => backend.AddConstraint(definition));
        Assert.Throws<ArgumentException>(() => backend.AddConstraint(definition with { BodyB = a }));
        Assert.Throws<ArgumentException>(() => backend.AddConstraint(definition with { MinimumDistance = 2, MaximumDistance = 1 }));
        Assert.That(backend.BodyCount, Is.EqualTo(2));
        Assert.That(backend.ConstraintCount, Is.Zero);
    }
}
