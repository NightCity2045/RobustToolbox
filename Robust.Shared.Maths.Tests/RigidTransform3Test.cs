using System;
using System.Numerics;
using NUnit.Framework;

namespace Robust.Shared.Maths.Tests;

[TestFixture]
public sealed class RigidTransform3Test
{
    [Test]
    public void NestedPitchAndYawComposeInParentOrder()
    {
        var parent = new RigidTransform3(new Vector3(10, -2, 5),
            Quaternion.CreateFromAxisAngle(Vector3.UnitZ, MathF.PI / 2));
        var child = new RigidTransform3(new Vector3(2, 0, 3),
            Quaternion.CreateFromAxisAngle(Vector3.UnitX, MathF.PI / 2));
        var world = parent.Compose(child);
        Near(world.Position, new Vector3(10, 0, 8));
        Near(world.TransformVector(Vector3.UnitX), Vector3.UnitY);
        Near(world.TransformVector(Vector3.UnitY), Vector3.UnitZ);
        Near(world.TransformPoint(Vector3.One), parent.TransformPoint(child.TransformPoint(Vector3.One)));
    }

    [TestCase(-100000f)]
    [TestCase(100000f)]
    public void MatrixAndInversePreserveLargeNegativeAndPositiveCoordinates(float offset)
    {
        var pose = new RigidTransform3(new Vector3(offset, -offset, offset),
            Quaternion.CreateFromYawPitchRoll(0.8f, -0.3f, 1.1f));
        var local = new Vector3(-31, 42, -17);
        var world = pose.TransformPoint(local);
        Near(world, Vector3.Transform(local, pose.ToMatrix()), 0.03f);
        Near(pose.InverseTransformPoint(world), local, 0.03f);
        Near(pose.Inverse().TransformPoint(world), local, 0.06f);
    }

    [Test]
    public void ReparentPreservesPositionAndAllThreeOrientationAxes()
    {
        var oldParent = new RigidTransform3(new Vector3(-21, 42, 13),
            Quaternion.CreateFromYawPitchRoll(0.3f, 0.8f, -1.2f));
        var newParent = new RigidTransform3(new Vector3(53, -8, -9),
            Quaternion.CreateFromYawPitchRoll(-1.1f, 0.2f, 0.7f));
        var child = new RigidTransform3(new Vector3(4, -7, 3.25f),
            Quaternion.CreateFromYawPitchRoll(0.9f, -0.5f, 0.4f));
        var before = oldParent.Compose(child);
        var after = newParent.Compose(newParent.ToLocal(before));
        Near(after.Position, before.Position);
        foreach (var axis in new[] { Vector3.UnitX, Vector3.UnitY, Vector3.UnitZ })
            Near(after.TransformVector(axis), before.TransformVector(axis));
    }

    [Test]
    public void MovingRotatingFrameAddsPointVelocityOnce()
    {
        var frame = new RigidTransform3(new Vector3(10, 0, 0),
            Quaternion.CreateFromAxisAngle(Vector3.UnitZ, MathF.PI / 2));
        var point = new Vector3(2, 0, 3);
        var relative = new Vector3(1, 0, -2);
        var linear = new Vector3(4, 0, 1);
        var angular = new Vector3(0, 0, 2);
        var world = frame.TransformPointVelocity(point, relative, linear, angular);
        Near(world, new Vector3(0, 1, -1));
        Near(frame.InverseTransformPointVelocity(point, world, linear, angular), relative);
    }

    [TestCase(float.MaxValue)]
    [TestCase(1e-30f)]
    public void ScaledFiniteQuaternionsNormalizeWithoutOverflowOrUnderflow(float magnitude)
    {
        var pose = new RigidTransform3(Vector3.Zero, new Quaternion(0, 0, magnitude, magnitude));
        Assert.That(pose.IsValid, Is.True);
        Near(pose.TransformVector(Vector3.UnitX), Vector3.UnitY);
        Assert.That(new RigidTransform3(Vector3.Zero, -pose.Rotation), Is.EqualTo(pose));
    }

    [TestCase(float.NaN, 0f, 0f)]
    [TestCase(0f, float.PositiveInfinity, 0f)]
    [TestCase(0f, 0f, float.NegativeInfinity)]
    public void NonFinitePositionIsRejected(float x, float y, float z)
    {
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            _ = new RigidTransform3(new Vector3(x, y, z), Quaternion.Identity));
    }

    [Test]
    public void InvalidRotationsAndUninitializedPosesAreRejected()
    {
        Assert.Throws<ArgumentOutOfRangeException>(() => _ = new RigidTransform3(Vector3.Zero, default));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            _ = new RigidTransform3(Vector3.Zero, new Quaternion(0, float.NaN, 0, 1)));
        Assert.That(default(RigidTransform3).IsValid, Is.False);
        Assert.Throws<InvalidOperationException>(() => default(RigidTransform3).TransformPoint(Vector3.One));
        Assert.Throws<ArithmeticException>(() =>
            new RigidTransform3(new Vector3(float.MaxValue, 0, 0), Quaternion.Identity)
                .TransformPoint(new Vector3(float.MaxValue, 0, 0)));
    }

    private static void Near(Vector3 actual, Vector3 expected, float tolerance = 0.0001f)
        => Assert.That(Vector3.Distance(actual, expected), Is.LessThanOrEqualTo(tolerance),
            $"Expected {expected}, got {actual}.");
}
