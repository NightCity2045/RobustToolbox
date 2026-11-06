using System.Numerics;
using NUnit.Framework;
using Robust.Shared.GameObjects;
using Robust.Shared.Maths;
using Robust.Shared.Spatial.Physics;

namespace Robust.Shared.Tests.Spatial;

[TestFixture]
public sealed class SpatialCompoundFilterTest
{
    [Test]
    public void RaysCastsAndManifoldsFilterEachPartAfterBroadphaseUnion()
    {
        using var backend = new BepuSpatialPhysicsBackend(Vector3.Zero);
        var near = new EntityUid(1);
        var far = new EntityUid(2);
        var parts = new[]
        {
            new SpatialBackendPart(new SpatialBox(Vector3.One), new RigidTransform3(Vector3.UnitX * 2, Quaternion.Identity), near),
            new SpatialBackendPart(new SpatialBox(Vector3.One), new RigidTransform3(Vector3.UnitX * 4, Quaternion.Identity), far,
                CollisionLayer: 2, CollisionMask: 4)
        };
        backend.AddBody(new SpatialBackendBodyDefinition(SpatialBackendMotion.Static, parts, collisionLayer: 1),
            new SpatialBackendState(RigidTransform3.Identity, default, default));
        Assert.That(backend.RayCast(Vector3.Zero, Vector3.UnitX, 10, 1)!.Value.Source.Entity, Is.EqualTo(near));
        Assert.That(backend.RayCast(Vector3.Zero, Vector3.UnitX, 10, 2)!.Value.Source.Entity, Is.EqualTo(far));
        Assert.That(backend.RayCast(Vector3.Zero, Vector3.UnitX, 10, 4), Is.Null);
        var shape = new SpatialSphere(0.1f);
        var hits = backend.ShapeCast(shape, RigidTransform3.Identity, Vector3.UnitX * 6, Vector3.Zero, 2);
        Assert.That(hits.Select(h => h.Source.Entity), Is.EqualTo(new[] { far }));
        var atNear = new RigidTransform3(Vector3.UnitX * 2, Quaternion.Identity);
        Assert.That(backend.OverlapContacts(shape, atNear, 2), Is.Empty);
        Assert.That(backend.OverlapContacts(shape, atNear, 1), Is.Not.Empty);
    }

    [TestCase(1u, false)]
    [TestCase(4u, true)]
    public void CompoundPartMasksControlActualContactsAndMotorSupport(uint layer, bool collides)
    {
        using var backend = new BepuSpatialPhysicsBackend(-Vector3.UnitZ * 9.81f);
        var parts = new[]
        {
            new SpatialBackendPart(new SpatialBox(new Vector3(10, 10, 0.2f)),
                new RigidTransform3(-Vector3.UnitZ * 0.1f, Quaternion.Identity), CollisionLayer: 2, CollisionMask: 4),
            // This distant part makes the root's union accept layer 1 as well.
            // Only the child-level mask can reject its contact at the near floor.
            new SpatialBackendPart(new SpatialBox(Vector3.One),
                new RigidTransform3(Vector3.UnitX * 100, Quaternion.Identity), CollisionLayer: 1, CollisionMask: 1)
        };
        backend.AddBody(new SpatialBackendBodyDefinition(SpatialBackendMotion.Static, parts),
            new SpatialBackendState(RigidTransform3.Identity, default, default));
        var motor = backend.AddBody(new SpatialBackendBodyDefinition(SpatialBackendMotion.Dynamic,
            new[] { new SpatialBackendPart(new SpatialCapsule(0.3f, 1), RigidTransform3.Identity) }, mass: 80,
            collisionLayer: layer, collisionMask: 2, lockRotation: true),
            new SpatialBackendState(new RigidTransform3(Vector3.UnitZ * 0.8f, Quaternion.Identity), default, default));
        Assert.That(backend.FindMotorSupport(motor, SpatialMotorSettings.Default).HasValue, Is.EqualTo(collides));
        for (var i = 0; i < 60; i++) backend.Step(1f / 60);
        var z = backend.GetState(motor).Pose.Position.Z;
        if (collides) Assert.That(z, Is.EqualTo(0.8f).Within(0.02));
        else Assert.That(z, Is.LessThan(-2));
    }
}
