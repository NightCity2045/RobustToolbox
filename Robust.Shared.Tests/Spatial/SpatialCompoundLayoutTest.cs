using System.Numerics;
using NUnit.Framework;
using Robust.Shared.GameObjects;
using Robust.Shared.Maths;
using Robust.Shared.Spatial.Physics;

namespace Robust.Shared.Tests.Spatial;

[TestFixture]
public sealed class SpatialCompoundLayoutTest
{
    [Test]
    public void FlattenedPartsRetainMassOriginIdentityAndFilter()
    {
        var root = new EntityUid(1);
        var child = new EntityUid(2);
        var rootDefinition = new SpatialBackendBodyDefinition(SpatialBackendMotion.Dynamic,
            new[] { new SpatialBackendPart(new SpatialSphere(0.25f), RigidTransform3.Identity) }, mass: 2, collisionLayer: 1);
        var childDefinition = new SpatialBackendBodyDefinition(SpatialBackendMotion.Dynamic,
            new[] { new SpatialBackendPart(new SpatialSphere(0.25f),
                new RigidTransform3(Vector3.UnitX, Quaternion.Identity), Revision: 7) }, mass: 6, collisionLayer: 4, collisionMask: 2);
        var composed = SpatialCompoundLayout.Compose(root, rootDefinition, new[]
        {
            new SpatialCompoundMember(child, new RigidTransform3(Vector3.UnitX * 4, Quaternion.Identity), childDefinition)
        });
        using var backend = new BepuSpatialPhysicsBackend(Vector3.Zero);
        var body = backend.AddBody(composed, new SpatialBackendState(RigidTransform3.Identity, default, default));
        backend.ApplyImpulse(body, Vector3.UnitY * 8, Vector3.UnitX * 3.75f); // The mass-weighted center.
        var state = backend.GetState(body);
        Assert.That(state.AngularVelocity.Length(), Is.LessThan(1e-5));
        Assert.That(Vector3.Distance(state.LinearVelocity, Vector3.UnitY), Is.LessThan(1e-5));
        var hit = backend.RayCast(Vector3.UnitX * 10, -Vector3.UnitX, 20, 4)!.Value;
        Assert.That(hit.Source.Entity, Is.EqualTo(child));
        Assert.That(hit.Source.Revision, Is.EqualTo(7));
        Assert.That(hit.Point.X, Is.EqualTo(5.25f).Within(1e-5));
        Assert.That(backend.RayCast(Vector3.UnitX * 10, -Vector3.UnitX, 20, 1)!.Value.Source.Entity, Is.EqualTo(root));
        backend.Step(0.1f);
        Assert.That(Vector3.Distance(backend.GetState(body).Pose.Position, Vector3.UnitY * 0.1f), Is.LessThan(1e-5));
    }

    [Test]
    public void InvalidMembershipAndMassAreRejectedBeforeAnyBackendMutation()
    {
        var root = new EntityUid(1);
        var shape = new[] { new SpatialBackendPart(new SpatialSphere(1), RigidTransform3.Identity) };
        var definition = new SpatialBackendBodyDefinition(SpatialBackendMotion.Dynamic, shape);
        var member = new SpatialCompoundMember(new EntityUid(2), RigidTransform3.Identity, definition);
        Assert.Throws<ArgumentException>(() => SpatialCompoundLayout.Compose(root, definition, new[] { member, member }));
        Assert.Throws<ArgumentException>(() => SpatialCompoundLayout.Compose(root, definition, new[] { member with { Entity = root } }));
        var huge = new SpatialBackendBodyDefinition(SpatialBackendMotion.Dynamic, shape, mass: float.MaxValue);
        Assert.Throws<ArgumentException>(() => SpatialCompoundLayout.Compose(root, huge, new[] { member with { Definition = huge } }));
    }
}
