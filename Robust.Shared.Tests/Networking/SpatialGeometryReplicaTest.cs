using System.IO;
using System.Numerics;
using System.Security.Cryptography;
using NUnit.Framework;
using Robust.Shared.GameObjects;
using Robust.Shared.Maths;
using Robust.Shared.Spatial;
using Robust.Shared.Spatial.Networking;
using Robust.Shared.Timing;

namespace Robust.Shared.Tests.Networking;

[TestFixture]
public sealed class SpatialGeometryReplicaTest
{
    private static readonly NetEntity World = new(99);
    private static readonly Box3 View = new(new Vector3(-10), new Vector3(10));

    private static (SpatialGeometryFrameManifest Description, byte[] Bytes) Payload(int frame, ulong revision,
        Guid? generation = null, bool large = false)
    {
        var uid = new EntityUid(frame);
        var layer = new LayerId(1);
        var slabs = Enumerable.Range(0, large ? 20 : 1).Select(i => new SlabSurface(
            new SpatialSurfaceReference(uid, new SurfaceId((uint) i)), layer, new Box2(i, 0, i + 1, 1),
            0, 0.2f, materialProfile: large ? new string('m', 2000) : "metal", stateRevision: revision));
        var snapshot = new SpatialGeometrySnapshot(uid, revision,
            new Dictionary<LayerId, LayerDefinition> { [layer] = new(0, "Floor") },
            Array.Empty<BoundarySegment>(), slabs, Array.Empty<SpatialTileChunk>());
        var bytes = SpatialGeometryWireCodec.Encode(snapshot, entity => new NetEntity((int) entity));
        return (new SpatialGeometryFrameManifest(World, new NetEntity(frame),
            new SpatialGeometryStamp(generation ?? Guid.NewGuid(), revision),
            Convert.ToHexString(SHA256.HashData(bytes)), bytes.Length), bytes);
    }

    private static SpatialGeometryManifest Manifest(ulong sequence, ulong baseline, uint tick,
        params SpatialGeometryFrameManifest[] frames)
        => new(sequence, baseline, new GameTick(tick), frames, new[] { new SpatialGeometryCoverage(World, View) });

    private static EntityUid? Resolve(NetEntity world, NetEntity uid)
        => world == World ? new EntityUid(uid.Id + 100) : null;

    private static void Send(SpatialGeometryReplica replica, ulong sequence,
        (SpatialGeometryFrameManifest Description, byte[] Bytes) payload, bool reverse = false)
    {
        var offsets = Enumerable.Range(0, (payload.Bytes.Length + SpatialGeometryReplica.FragmentSize - 1) / SpatialGeometryReplica.FragmentSize)
            .Select(i => i * SpatialGeometryReplica.FragmentSize);
        if (reverse) offsets = offsets.Reverse();
        foreach (var offset in offsets)
            Assert.That(replica.AddFragment(sequence, payload.Description.Frame, offset,
                payload.Bytes.AsSpan(offset, Math.Min(SpatialGeometryReplica.FragmentSize, payload.Bytes.Length - offset))), Is.True);
    }

    [Test]
    public void MultiFrameInstallWaitsForEveryFragmentReferenceAndAuthoritativeTick()
    {
        var replica = new SpatialGeometryReplica();
        var a = Payload(1, 0);
        var b = Payload(2, 3, large: true);
        var expected = new Dictionary<EntityUid, SpatialGeometryStamp>
        {
            [new EntityUid(101)] = a.Description.Stamp,
            [new EntityUid(102)] = b.Description.Stamp
        };
        Assert.That(replica.Begin(Manifest(1, 0, 20, a.Description, b.Description)), Is.True);
        Send(replica, 1, a);
        Assert.That(replica.TryApply(new GameTick(20), Resolve, uid => expected[uid]), Is.EqualTo(SpatialGeometryApplyResult.Waiting));
        Assert.That(replica.TryGetSnapshot(a.Description.Frame, out _), Is.False);
        Assert.That(replica.IsCovered(World, View), Is.False);
        Send(replica, 1, b, reverse: true);
        Send(replica, 1, b); // Identical retransmission is harmless.
        Assert.That(replica.TryApply(new GameTick(19), Resolve, uid => expected[uid]), Is.EqualTo(SpatialGeometryApplyResult.Waiting));
        Assert.That(replica.TryApply(new GameTick(20), (world, uid) => uid.Id == 2 ? null : Resolve(world, uid),
            uid => expected[uid]), Is.EqualTo(SpatialGeometryApplyResult.Waiting));
        Assert.That(replica.TryGetSnapshot(a.Description.Frame, out _), Is.False);
        Assert.That(replica.TryApply(new GameTick(20), Resolve, uid => expected[uid]), Is.EqualTo(SpatialGeometryApplyResult.Applied));
        Assert.That(replica.IsCovered(World, View), Is.True);
        Assert.That(replica.TryGetSnapshot(a.Description.Frame, out var snapshot), Is.True);
        Assert.That(snapshot!.OwnerFrame, Is.EqualTo(new EntityUid(101)));
        Assert.That(replica.Sequence, Is.EqualTo(1));
    }

    [Test]
    public void DeltaReusesUnchangedFramesAndAtomicallyReplacesChangedOnes()
    {
        var replica = new SpatialGeometryReplica();
        var a = Payload(1, 0);
        var b = Payload(2, 0);
        replica.Begin(Manifest(1, 0, 1, a.Description, b.Description));
        Send(replica, 1, a); Send(replica, 1, b);
        replica.TryApply(new GameTick(1), Resolve, uid => uid == new EntityUid(101) ? a.Description.Stamp : b.Description.Stamp);
        replica.TryGetSnapshot(b.Description.Frame, out var originalB);
        var changed = Payload(1, 1, a.Description.Stamp.Generation);
        replica.Begin(Manifest(2, 1, 2, changed.Description, b.Description));
        Assert.That(replica.IsCovered(World, View), Is.False);
        Assert.That(replica.TryGetSnapshot(a.Description.Frame, out _), Is.False);
        Assert.That(replica.TryGetSnapshot(b.Description.Frame, out _), Is.True);
        Send(replica, 2, changed);
        Assert.That(replica.TryApply(new GameTick(2), Resolve,
            uid => uid == new EntityUid(101) ? changed.Description.Stamp : b.Description.Stamp), Is.EqualTo(SpatialGeometryApplyResult.Applied));
        replica.TryGetSnapshot(b.Description.Frame, out var currentB);
        Assert.That(currentB, Is.SameAs(originalB));
        Assert.That(replica.IsCovered(World, View), Is.True);
    }

    [Test]
    public void MissingBaselineAndRecreatedComponentCannotAcceptStaleGeometry()
    {
        var replica = new SpatialGeometryReplica();
        var a = Payload(1, 0);
        replica.Begin(Manifest(1, 0, 1, a.Description)); Send(replica, 1, a);
        replica.TryApply(new GameTick(1), Resolve, _ => a.Description.Stamp);
        Assert.That(replica.Begin(Manifest(3, 2, 3, a.Description)), Is.False);
        Assert.That(replica.NeedsBaseline, Is.True);
        Assert.That(replica.IsCovered(World, View), Is.False);
        Assert.That(replica.Begin(Manifest(4, 0, 4, a.Description)), Is.True);
        Assert.That(replica.TryApply(new GameTick(4), Resolve, _ => a.Description.Stamp), Is.EqualTo(SpatialGeometryApplyResult.Waiting),
            "A full baseline must consume all fragments even when identical bytes were cached.");
        Send(replica, 4, a);
        Assert.That(replica.TryApply(new GameTick(4), Resolve, _ => a.Description.Stamp), Is.EqualTo(SpatialGeometryApplyResult.Applied));
        Assert.That(replica.Begin(Manifest(2, 0, 2, a.Description)), Is.False, "An old full baseline cannot roll back the current one.");
        replica.Invalidate(a.Description.Frame);
        Assert.That(replica.IsCovered(World, View), Is.False);
        replica.Begin(Manifest(5, 4, 5, a.Description));
        Assert.That(replica.TryApply(new GameTick(5), Resolve, _ => new SpatialGeometryStamp(Guid.NewGuid(), 0)),
            Is.EqualTo(SpatialGeometryApplyResult.NeedBaseline));
        replica.Clear();
        Assert.That(replica.Sequence, Is.Zero);
        Assert.That(replica.IsCovered(World, View), Is.False);
        Assert.That(replica.TryGetSnapshot(a.Description.Frame, out _), Is.False);
    }

    [Test]
    public void NewFrameStateInvalidatesEmptyCoverageAndOlderInFlightManifests()
    {
        var replica = new SpatialGeometryReplica();
        replica.Begin(Manifest(1, 0, 1));
        Assert.That(replica.TryApply(new GameTick(1), Resolve, _ => null), Is.EqualTo(SpatialGeometryApplyResult.Applied));
        replica.Begin(Manifest(2, 1, 2));
        var added = Payload(2, 0);
        replica.Invalidate(added.Description.Frame, new GameTick(3));
        Assert.That(replica.IsCovered(World, View), Is.False);
        Assert.That(replica.TryApply(new GameTick(3), Resolve, _ => null), Is.EqualTo(SpatialGeometryApplyResult.NeedBaseline));
        replica.Begin(Manifest(3, 0, 3, added.Description));
        Send(replica, 3, added);
        Assert.That(replica.TryApply(new GameTick(3), Resolve, _ => added.Description.Stamp), Is.EqualTo(SpatialGeometryApplyResult.Applied));
        Assert.That(replica.IsCovered(World, View), Is.True);
        replica.Invalidate(added.Description.Frame, new GameTick(4));
        Assert.That(replica.TryApply(new GameTick(4), Resolve, _ => null), Is.EqualTo(SpatialGeometryApplyResult.NeedBaseline));
        // A newer manifest may certify that a removed/out-of-range frame no
        // longer contributes any geometry to this authorized volume.
        replica.Begin(Manifest(4, 0, 4));
        Assert.That(replica.TryApply(new GameTick(4), Resolve, _ => null), Is.EqualTo(SpatialGeometryApplyResult.Applied));
        Assert.That(replica.IsCovered(World, View), Is.True);
    }

    [Test]
    public void CorruptionOrConflictingRetransmissionCannotLeaveApplicablePartialData()
    {
        var replica = new SpatialGeometryReplica();
        var a = Payload(1, 0);
        replica.Begin(Manifest(1, 0, 1, a.Description));
        var changed = a.Bytes.ToArray(); changed[^1] ^= 1;
        Assert.Throws<InvalidDataException>(() => replica.AddFragment(1, a.Description.Frame, 0, changed));
        Assert.That(replica.NeedsBaseline, Is.True);
        Assert.That(replica.HasPending, Is.False);
        Assert.That(replica.TryApply(new GameTick(1), Resolve, _ => a.Description.Stamp), Is.EqualTo(SpatialGeometryApplyResult.NeedBaseline));
        replica.Begin(Manifest(2, 0, 2, a.Description)); Send(replica, 2, a);
        Assert.Throws<InvalidDataException>(() => replica.AddFragment(2, a.Description.Frame, 0, changed));
        Assert.That(replica.HasPending, Is.False);
    }

    [Test]
    public void SameRevisionInAnotherWorldInvalidatesCachedCollisionBeforeNextManifest()
    {
        var replica = new SpatialGeometryReplica();
        var a = Payload(1, 0);
        replica.Begin(Manifest(1, 0, 1, a.Description));
        Send(replica, 1, a);
        replica.TryApply(new GameTick(1), Resolve, _ => a.Description.Stamp);
        var destination = new NetEntity(98);
        EntityUid? Moved(NetEntity world, NetEntity uid) => world == destination ? new EntityUid(uid.Id + 100) : null;
        replica.InvalidateUnresolved(Moved, new GameTick(2));
        Assert.That(replica.TryGetSnapshot(a.Description.Frame, out _), Is.False);
        Assert.That(replica.IsCovered(World, View), Is.False);
        Assert.That(replica.IsCovered(destination, View), Is.False);
        var description = a.Description with { World = destination };
        replica.Begin(new SpatialGeometryManifest(2, 1, new GameTick(2), new[] { description },
            new[] { new SpatialGeometryCoverage(destination, View) }));
        Send(replica, 2, (description, a.Bytes));
        Assert.That(replica.TryApply(new GameTick(2), Moved, _ => a.Description.Stamp), Is.EqualTo(SpatialGeometryApplyResult.Applied));
        Assert.That(replica.IsCovered(destination, View), Is.True);
        Assert.That(replica.IsCovered(World, View), Is.False);
    }

    [Test]
    public void ManifestBudgetsAndEmptyWorldCoverageAreExplicit()
    {
        var a = Payload(1, 0);
        Assert.Throws<InvalidDataException>(() => Manifest(1, 0, 1, a.Description, a.Description));
        Assert.Throws<InvalidDataException>(() => Manifest(1, 0, 1, a.Description with { Length = int.MaxValue }));
        var replica = new SpatialGeometryReplica();
        replica.Begin(Manifest(1, 0, 1));
        Assert.That(replica.IsCovered(World, View), Is.False);
        Assert.That(replica.TryApply(new GameTick(1), (_, _) => null, _ => null), Is.EqualTo(SpatialGeometryApplyResult.Waiting));
        Assert.That(replica.TryApply(new GameTick(1), Resolve, _ => null), Is.EqualTo(SpatialGeometryApplyResult.Applied));
        Assert.That(replica.IsCovered(World, View), Is.True);
        Assert.That(replica.IsCovered(new NetEntity(98), View), Is.False);
        Assert.That(replica.IsCovered(World, new Box3(new Vector3(-20), new Vector3(20))), Is.False);
    }
}
