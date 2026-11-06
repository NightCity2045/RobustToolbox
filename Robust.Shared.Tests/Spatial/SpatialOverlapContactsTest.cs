using System.Numerics;
using NUnit.Framework;
using Robust.Shared.GameObjects;
using Robust.Shared.Maths;
using Robust.Shared.Spatial;
using Robust.Shared.Spatial.Physics;

namespace Robust.Shared.Tests.Spatial;

[TestFixture]
public sealed class SpatialOverlapContactsTest
{
    [Test]
    public void ManifoldDistinguishesFloorTouchFromPenetrationWithoutChangingTheScene()
    {
        using var backend = new BepuSpatialPhysicsBackend(Vector3.Zero);
        var entity = new EntityUid(5);
        var surface = new SpatialSurfaceReference(entity, new SurfaceId(7));
        var part = new SpatialBackendPart(new SpatialBox(new Vector3(10, 10, 0.2f)), RigidTransform3.Identity,
            entity, surface, 12, MaterialProfile: "floor");
        var floor = backend.AddBody(new SpatialBackendBodyDefinition(SpatialBackendMotion.Static, new[] { part }),
            new SpatialBackendState(new RigidTransform3(new Vector3(0, 0, -0.1f), Quaternion.Identity), default, default));
        var initial = backend.GetState(floor);
        var sphere = new SpatialSphere(0.5f);
        var touching = backend.OverlapContacts(sphere, new RigidTransform3(new Vector3(0, 0, 0.5f), Quaternion.Identity), uint.MaxValue);
        Assert.That(touching.SelectMany(o => o.Contacts).All(c => c.Depth <= 1e-6), Is.True);
        var overlap = backend.OverlapContacts(sphere, new RigidTransform3(new Vector3(0, 0, 0.45f), Quaternion.Identity), uint.MaxValue);
        Assert.That(overlap, Has.Count.EqualTo(1));
        Assert.That(overlap[0].Source, Is.EqualTo(part));
        var witness = overlap[0].Contacts.Single();
        Assert.That(witness.Depth, Is.EqualTo(0.05f).Within(1e-5));
        Assert.That(witness.Normal.Z, Is.GreaterThan(0.99));
        Assert.That(witness.Point.Z, Is.InRange(-0.06f, 0.01f));
        Assert.That(backend.OverlapContacts(sphere, new RigidTransform3(new Vector3(0, 0, 0.6f), Quaternion.Identity), uint.MaxValue), Is.Empty);
        Assert.That(backend.OverlapContacts(sphere, new RigidTransform3(new Vector3(0, 0, 0.45f), Quaternion.Identity), uint.MaxValue, floor), Is.Empty);
        Assert.That(backend.GetState(floor), Is.EqualTo(initial));
        Assert.That(backend.BodyCount, Is.EqualTo(1));
        Assert.That(backend.DrainContactEvents(), Is.Empty);
    }

    [TestCase(false)]
    [TestCase(true)]
    public void CapsuleManifoldUsesZAxisAndBothWorldOrientations(bool rotateCapsule)
    {
        using var backend = new BepuSpatialPhysicsBackend(Vector3.Zero);
        var rotation = Quaternion.CreateFromAxisAngle(Vector3.UnitY, MathF.PI / 2);
        backend.AddBody(new SpatialBackendBodyDefinition(SpatialBackendMotion.Static,
            new[] { new SpatialBackendPart(new SpatialBox(new Vector3(10, 10, 0.2f)),
                new RigidTransform3(new Vector3(0, 0, -0.1f), Quaternion.Identity)) }),
            new SpatialBackendState(new RigidTransform3(new Vector3(4, 3, 2), rotation), default, default));
        var pose = new RigidTransform3(new Vector3(4 + (rotateCapsule ? 0.7f : 0.2f), 3, 2),
            rotateCapsule ? rotation : Quaternion.Identity);
        var result = backend.OverlapContacts(new SpatialCapsule(0.25f, 1), pose, uint.MaxValue);
        Assert.That(result, Has.Count.EqualTo(1));
        Assert.That(result[0].Contacts.Max(c => c.Depth), Is.EqualTo(0.05f).Within(1e-4));
        foreach (var contact in result[0].Contacts)
        {
            Assert.That(contact.Normal.X, Is.GreaterThan(0.99));
            Assert.That(contact.Point.X, Is.InRange(3.94f, 4.01f));
        }
    }

    [Test]
    public void BatchedContactsKeepAllChildSourcesAcrossInternalFlushes()
    {
        using var backend = new BepuSpatialPhysicsBackend(Vector3.Zero);
        var frame = new EntityUid(7);
        var parts = Enumerable.Range(0, 64).Select(i => new SpatialBackendPart(new SpatialSphere(0.2f),
            new RigidTransform3(new Vector3(i - 32, 0, 0), Quaternion.Identity), frame,
            new SpatialSurfaceReference(frame, new SurfaceId((uint) i)), (ulong) i)).ToArray();
        backend.AddBody(new SpatialBackendBodyDefinition(SpatialBackendMotion.Static, parts),
            new SpatialBackendState(RigidTransform3.Identity, default, default));
        for (var iteration = 0; iteration < 4; iteration++)
        {
            var overlaps = backend.OverlapContacts(new SpatialBox(new Vector3(80, 4, 4)), RigidTransform3.Identity, uint.MaxValue);
            Assert.That(overlaps, Has.Count.EqualTo(64));
            Assert.That(overlaps.Select(o => o.Source.Revision).Order(), Is.EqualTo(Enumerable.Range(0, 64).Select(i => (ulong) i)));
            Assert.That(overlaps.All(o => o.Contacts.Count > 0 && o.Contacts.All(c => c.Depth > 0)), Is.True);
        }
        Assert.That(backend.BodyCount, Is.EqualTo(1));
        Assert.That(backend.DrainContactEvents(), Is.Empty);
    }
}
