using System.Numerics;
using NUnit.Framework;
using Robust.Client.Graphics.Spatial;
using Robust.Shared.GameObjects;
using Robust.Shared.Maths;

namespace Robust.Client.Tests.Graphics;

[TestFixture]
public sealed class SpatialParticleTest
{
    private static readonly PerspectiveCamera Camera = new(new RigidTransform3(new Vector3(0, -10, 5), Quaternion.Identity), MathF.PI / 3, 1, 0.1f, 100);
    private static readonly Dictionary<EntityUid, RigidTransform3> Frames = new();
    private static SpatialParticleEmission Emission => new(
        new SpatialSpriteProfile(SpatialSpriteKind.DirectionalBillboard, Vector2.One, new Vector2(0.5f), MathF.PI / 2),
        new RigidTransform3(new Vector3(1, 2, 3), Quaternion.Identity), "particle", 4,
        new Vector3(2, 0, 4), new Vector3(0, 0, -2), new Vector2(2), Vector4.One, new Vector4(1, 0, 0, 0));

    [Test]
    public void XYZMotionSizeAndColorAreIndependentOfObserverAndStepPartition()
    {
        using var once = new SpatialParticlePool(new SpatialParticleLimits(4, 10));
        using var many = new SpatialParticlePool(new SpatialParticleLimits(4, 10));
        once.TryEmit(Emission, out _); many.TryEmit(Emission, out _);
        once.Advance(2);
        for (var i = 0; i < 8; i++) many.Advance(0.25);
        var first = once.CreateVisuals(Camera, Frames).Single();
        var second = many.CreateVisuals(Camera, Frames).Single();
        Assert.That(first.Pose, Is.EqualTo(second.Pose));
        Assert.That(first.Pose.Position, Is.EqualTo(new Vector3(5, 2, 7)));
        Assert.That(first.Size, Is.EqualTo(new Vector3(1.5f, 1.5f, 1)));
        Assert.That(first.Modulate, Is.EqualTo(new Vector4(1, 0.5f, 0.5f, 0.5f)));
        var opposite = new PerspectiveCamera(new RigidTransform3(new Vector3(0, 10, 5), Quaternion.Identity), MathF.PI / 3, 1, 0.1f, 100);
        var other = once.CreateVisuals(opposite, Frames).Single();
        Assert.That(other.Pose.Position, Is.EqualTo(first.Pose.Position));
        Assert.That(other.Pose.Rotation, Is.Not.EqualTo(first.Pose.Rotation));
        once.Advance(2);
        Assert.That(once.Count, Is.Zero);
    }

    [Test]
    public void AttachedParticlesFollowCurrentFrameAndRevocationExpiresThem()
    {
        var frame = new EntityUid(1); var owner = new EntityUid(2);
        using var pool = new SpatialParticlePool(new SpatialParticleLimits(4, 10));
        pool.TryEmit(Emission with { Frame = frame, Owner = owner }, out _);
        pool.Advance(1);
        var pose = new RigidTransform3(new Vector3(10, 20, 30), Quaternion.CreateFromYawPitchRoll(0.4f, 0.2f, 1));
        var frames = new Dictionary<EntityUid, RigidTransform3> { [frame] = pose };
        Assert.That(Vector3.Distance(pool.CreateVisuals(Camera, frames).Single().Pose.Position,
            pose.TransformPoint(new Vector3(3, 2, 6))), Is.LessThan(1e-5));
        Assert.That(pool.CreateVisuals(Camera, Frames), Is.Empty);
        Assert.That(pool.Count, Is.Zero);
        pool.TryEmit(Emission with { Owner = owner }, out _);
        pool.RemoveOwner(owner);
        Assert.That(pool.Count, Is.Zero);
    }

    [Test]
    public void BudgetExpiryAndInvalidMotionNeverCreateUnboundedOrNonfiniteVisuals()
    {
        using var pool = new SpatialParticlePool(new SpatialParticleLimits(1, 10));
        Assert.That(pool.TryEmit(Emission, out var first), Is.True);
        Assert.That(pool.TryEmit(Emission, out var rejected), Is.False);
        Assert.That(rejected, Is.Zero); Assert.That(pool.RejectedByBudget, Is.EqualTo(1));
        pool.Remove(first);
        pool.TryEmit(Emission with { Velocity = new Vector3(float.MaxValue, 0, 0) }, out _);
        pool.Advance(2);
        Assert.That(pool.CreateVisuals(Camera, Frames), Is.Empty); Assert.That(pool.InvalidMotion, Is.EqualTo(1));
        Assert.Throws<ArgumentException>(() => pool.TryEmit(Emission with { Lifetime = 11 }, out _));
        Assert.Throws<ArgumentException>(() => pool.TryEmit(Emission with { Acceleration = new Vector3(float.NaN) }, out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => pool.Advance(-1));
        pool.Dispose();
        Assert.Throws<ObjectDisposedException>(() => pool.Advance(1));
    }
}
