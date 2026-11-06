using System;
using System.Numerics;
using NUnit.Framework;
using Robust.Shared.GameObjects;
using Robust.Shared.Maths;
using Robust.Shared.Spatial.Networking;
using Robust.Shared.Timing;

namespace Robust.Shared.Tests.Spatial;

[TestFixture]
public sealed class SpatialPoseTimelineTest
{
    private static readonly EntityUid World = new(1), Carrier = new(2), Cargo = new(3), Passenger = new(4);
    private static SpatialPoseNode[] Frame(float angle) => new[]
    {
        new SpatialPoseNode(Cargo, Carrier, new RigidTransform3(Vector3.UnitX, Quaternion.Identity)),
        new SpatialPoseNode(Passenger, Carrier, new RigidTransform3(
            Vector3.Transform(Vector3.UnitX, Quaternion.CreateFromAxisAngle(Vector3.UnitZ, -angle)),
            Quaternion.CreateFromAxisAngle(Vector3.UnitZ, -angle)), SpatialPoseInterpolation.World),
        new SpatialPoseNode(Carrier, World, new RigidTransform3(Vector3.Zero, Quaternion.CreateFromAxisAngle(Vector3.UnitZ, angle)), SpatialPoseInterpolation.World),
        new SpatialPoseNode(World, null, RigidTransform3.Identity),
    };

    [Test]
    public void AnchoredMembersFollowTheArcAndIndependentPassengersDoNotMoveTwice()
    {
        var timeline = new SpatialPoseTimeline(World, new SpatialPoseTimelineLimits(4, 8, 32));
        timeline.Install(new GameTick(10), Frame(0));
        timeline.Install(new GameTick(20), Frame(MathF.PI / 2));
        Assert.That(timeline.TrySample(Cargo, 15, out var cargo), Is.True);
        Assert.That(Vector3.Distance(cargo.Position, new Vector3(MathF.Sqrt(0.5f), MathF.Sqrt(0.5f), 0)), Is.LessThan(1e-5));
        Assert.That(timeline.TrySample(Passenger, 15, out var passenger), Is.True);
        Assert.That(Vector3.Distance(passenger.Position, Vector3.UnitX), Is.LessThan(1e-5));
        Assert.That(Vector3.Distance(passenger.TransformVector(Vector3.UnitY), Vector3.UnitY), Is.LessThan(1e-5));
        Assert.That(timeline.TrySample(Cargo, 9, out _), Is.False);
        Assert.That(timeline.TrySample(Cargo, 100, out cargo), Is.True);
        Assert.That(Vector3.Distance(cargo.Position, Vector3.UnitY), Is.LessThan(1e-5));
        Assert.That(timeline.Install(new GameTick(19), Frame(0)), Is.EqualTo(SpatialPoseInstallResult.Older));
    }

    [Test]
    public void RevocationBudgetsAndParentChangesNeverInventAnOriginOrReuseOldPresentation()
    {
        var timeline = new SpatialPoseTimeline(World, new SpatialPoseTimelineLimits(2, 8, 8));
        timeline.Install(new GameTick(10), Frame(0));
        var changed = Frame(0);
        changed[0] = changed[0] with { Parent = World, LocalPose = new RigidTransform3(new Vector3(10, 20, 30), Quaternion.Identity) };
        timeline.Install(new GameTick(20), changed);
        Assert.That(timeline.TrySample(Cargo, 10, out var pose), Is.True);
        Assert.That(pose.Position, Is.EqualTo(Vector3.UnitX), "An exact sample precedes the reparent.");
        Assert.That(timeline.TrySample(Cargo, 15, out pose), Is.True);
        Assert.That(pose.Position, Is.EqualTo(new Vector3(10, 20, 30)));
        timeline.Invalidate(Carrier);
        Assert.That(timeline.TrySample(Passenger, 20, out _), Is.False);
        Assert.That(timeline.TrySample(Cargo, 15, out _), Is.False, "Revocation removed its old dependent sample.");
        Assert.That(timeline.TrySample(Cargo, 20, out _), Is.True);
        timeline.Install(new GameTick(30), Frame(0));
        timeline.Install(new GameTick(40), Frame(0));
        Assert.That(timeline.FrameCount, Is.EqualTo(2)); Assert.That(timeline.RetainedSamples, Is.EqualTo(8));
        Assert.That(timeline.TrySample(Cargo, 20, out _), Is.False);
        var limited = new SpatialPoseTimeline(World, new SpatialPoseTimelineLimits(2, 3, 3));
        limited.Install(new GameTick(1), new[] { new SpatialPoseNode(World, null, RigidTransform3.Identity) });
        Assert.That(limited.Install(new GameTick(2), Frame(0)), Is.EqualTo(SpatialPoseInstallResult.BudgetExceeded));
        Assert.That(limited.TrySample(World, 1, out _), Is.False);
        Assert.That(limited.RetainedSamples, Is.Zero);
    }

    [Test]
    public void MalformedHierarchyAndTickResetRequireExplicitRecovery()
    {
        var timeline = new SpatialPoseTimeline(World, new SpatialPoseTimelineLimits(3, 8, 24));
        var bad = Frame(0); bad[2] = bad[2] with { Parent = Cargo };
        Assert.Throws<ArgumentException>(() => timeline.Install(new GameTick(10), bad));
        bad = Frame(0); bad[2] = bad[2] with { Parent = new EntityUid(99) };
        Assert.Throws<ArgumentException>(() => timeline.Install(new GameTick(10), bad));
        timeline.Install(new GameTick(10), Frame(0));
        Assert.That(timeline.TrySample(Cargo, double.NaN, out _), Is.False);
        Assert.That(timeline.Install(new GameTick(1), Frame(0)), Is.EqualTo(SpatialPoseInstallResult.Older));
        timeline.Clear();
        Assert.That(timeline.Install(new GameTick(1), Frame(0)), Is.EqualTo(SpatialPoseInstallResult.Installed));
    }

    [Test]
    public void AncestorOwnershipDiscontinuitySnapsTheEntireDescendantPose()
    {
        var timeline = new SpatialPoseTimeline(World, new SpatialPoseTimelineLimits(3, 8, 24));
        timeline.Install(new GameTick(10), Frame(0));
        var next = Frame(MathF.PI / 2);
        next[2] = next[2] with { Interpolation = SpatialPoseInterpolation.Hierarchy };
        next[0] = next[0] with { LocalPose = new RigidTransform3(new Vector3(2, 0, 3), Quaternion.Identity) };
        timeline.Install(new GameTick(20), next);
        Assert.That(timeline.TrySample(Cargo, 10, out var old), Is.True);
        Assert.That(old.Position, Is.EqualTo(Vector3.UnitX));
        Assert.That(timeline.TrySample(Cargo, 15, out var changed), Is.True);
        Assert.That(timeline.TrySample(Cargo, 20, out var latest), Is.True);
        Assert.That(changed, Is.EqualTo(latest), "Do not mix the new ancestor with an old descendant-local pose.");
    }

    [Test]
    public void NewEntitiesAndRevokedParentsCannotAppearInEarlierOrPartialBatches()
    {
        var timeline = new SpatialPoseTimeline(World, new SpatialPoseTimelineLimits(3, 8, 24));
        timeline.Install(new GameTick(10), new[] { new SpatialPoseNode(World, null, RigidTransform3.Identity) });
        timeline.Install(new GameTick(20), Frame(0));
        Assert.That(timeline.TrySample(Cargo, 15, out _), Is.False);
        Assert.That(timeline.TrySample(Cargo, 20, out _), Is.True);
        timeline.Invalidate(World);
        Assert.That(timeline.RetainedSamples, Is.Zero);
        Assert.That(timeline.TrySample(Cargo, 20, out _), Is.False);
        timeline.Install(new GameTick(30), Frame(0));
        Assert.That(timeline.TrySample(Cargo, 25, out _), Is.False);
        Assert.That(timeline.TrySample(Cargo, 30, out _), Is.True);
    }
}
