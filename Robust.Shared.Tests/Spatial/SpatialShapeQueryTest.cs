using System.Numerics;
using NUnit.Framework;
using Robust.Shared.GameObjects;
using Robust.Shared.Maths;
using Robust.Shared.Spatial;
using Robust.Shared.Spatial.Physics;

namespace Robust.Shared.Tests.Spatial;

[TestFixture]
public sealed class SpatialShapeQueryTest
{
    private static RigidTransform3 Pose(Vector3 p) => new(p, Quaternion.Identity);

    [TestCase(0.01f)]
    [TestCase(10f)]
    public void SweepRetainsSourceAndFractionAtDifferentDistances(float distance)
    {
        using var backend = new BepuSpatialPhysicsBackend(Vector3.Zero);
        var source = new EntityUid(5);
        var surface = new SpatialSurfaceReference(new EntityUid(4), new SurfaceId(7));
        var part = new SpatialBackendPart(new SpatialBox(new Vector3(0.002f, 2, 2)),
            Pose(new Vector3(distance * 0.7f, 0, 0)), source, surface, 23);
        var body = backend.AddBody(new SpatialBackendBodyDefinition(SpatialBackendMotion.Static, new[] { part }),
            new SpatialBackendState(RigidTransform3.Identity, default, default));
        var hits = backend.ShapeCast(new SpatialSphere(0.001f), RigidTransform3.Identity,
            Vector3.UnitX * distance, Vector3.Zero, uint.MaxValue);
        Assert.That(hits, Has.Count.EqualTo(1));
        Assert.That(hits[0].Body, Is.EqualTo(body));
        Assert.That(hits[0].Source, Is.EqualTo(part));
        Assert.That(hits[0].Fraction, Is.EqualTo((distance * 0.7f - 0.002f) / distance).Within(0.002));
        Assert.That(hits[0].Normal!.Value.X, Is.LessThan(-0.99));
        Assert.That(hits[0].Point!.Value.X, Is.EqualTo(distance * 0.7f - 0.001f).Within(0.0001));
        Assert.That(backend.ShapeCast(new SpatialSphere(0.001f), RigidTransform3.Identity,
            Vector3.UnitX * distance, Vector3.Zero, uint.MaxValue, body), Is.Empty);
        Assert.That(backend.Overlap(new SpatialSphere(0.1f), Pose(new Vector3(distance * 0.7f, 0, 3)), uint.MaxValue), Is.Empty);
    }

    [Test]
    public void OverlapUsesActualShapeAndReportsInitialIntersectionWithoutInventedWitness()
    {
        using var backend = new BepuSpatialPhysicsBackend(Vector3.Zero);
        backend.AddBody(new SpatialBackendBodyDefinition(SpatialBackendMotion.Static,
            new[] { new SpatialBackendPart(new SpatialSphere(1), RigidTransform3.Identity) }),
            new SpatialBackendState(RigidTransform3.Identity, default, default));
        // The AABBs overlap at the corner, the spheres do not.
        Assert.That(backend.Overlap(new SpatialSphere(1), Pose(new Vector3(1.5f, 1.5f, 0)), uint.MaxValue), Is.Empty);
        var overlaps = backend.Overlap(new SpatialSphere(1), Pose(Vector3.UnitX), uint.MaxValue);
        Assert.That(overlaps, Has.Count.EqualTo(1));
        Assert.That(overlaps[0].Fraction, Is.Zero);
        Assert.That(overlaps[0].Point, Is.Null);
        Assert.That(overlaps[0].Normal, Is.Null);
    }

    [Test]
    public void RotationOnlySweepFindsThinObstacleOutsideInitialBounds()
    {
        using var backend = new BepuSpatialPhysicsBackend(Vector3.Zero);
        backend.AddBody(new SpatialBackendBodyDefinition(SpatialBackendMotion.Static,
            new[] { new SpatialBackendPart(new SpatialSphere(0.1f), RigidTransform3.Identity) }),
            new SpatialBackendState(Pose(new Vector3(0, 1.5f, 0)), default, default));
        var shape = new SpatialBox(new Vector3(4, 0.05f, 0.05f));
        Assert.That(backend.Overlap(shape, RigidTransform3.Identity, uint.MaxValue), Is.Empty);
        var hits = backend.ShapeCast(shape, RigidTransform3.Identity, Vector3.Zero,
            Vector3.UnitZ * MathF.PI, uint.MaxValue);
        Assert.That(hits, Has.Count.EqualTo(1));
        Assert.That(hits[0].Fraction, Is.InRange(0.45f, 0.5f));
    }

    [Test]
    public void CapsuleQueryUsesZAxisAndCurrentRotatedCompoundPose()
    {
        using var backend = new BepuSpatialPhysicsBackend(Vector3.Zero);
        var body = backend.AddBody(new SpatialBackendBodyDefinition(SpatialBackendMotion.Dynamic,
            new[] { new SpatialBackendPart(new SpatialBox(new Vector3(2, 0.2f, 0.2f)), Pose(new Vector3(3, 0, 0))) }),
            new SpatialBackendState(new RigidTransform3(Vector3.Zero,
                Quaternion.CreateFromAxisAngle(Vector3.UnitZ, MathF.PI / 2)), default, default));
        var capsule = new SpatialCapsule(0.2f, 2);
        Assert.That(backend.Overlap(capsule, Pose(new Vector3(0, 3, 1)), uint.MaxValue), Has.Count.EqualTo(1));
        Assert.That(backend.Overlap(capsule, Pose(new Vector3(0, 3, 2)), uint.MaxValue), Is.Empty);
        backend.SetState(body, new SpatialBackendState(Pose(new Vector3(20, 0, 0)), default, default));
        Assert.That(backend.Overlap(capsule, Pose(new Vector3(0, 3, 1)), uint.MaxValue), Is.Empty);
        backend.RemoveBody(body);
        Assert.That(backend.Overlap(capsule, Pose(new Vector3(23, 0, 1)), uint.MaxValue), Is.Empty);
    }

    [Test]
    public void QueryReturnsEverySourceInDistanceOrderAndHonorsMask()
    {
        using var backend = new BepuSpatialPhysicsBackend(Vector3.Zero);
        var parts = new[] { 4f, 2f, 8f }.Select(x => new SpatialBackendPart(new SpatialSphere(0.25f),
            Pose(new Vector3(x, 0, 0)), new EntityUid((int) x))).ToArray();
        backend.AddBody(new SpatialBackendBodyDefinition(SpatialBackendMotion.Static, parts, collisionLayer: 4),
            new SpatialBackendState(RigidTransform3.Identity, default, default));
        var hits = backend.ShapeCast(new SpatialSphere(0.25f), RigidTransform3.Identity, Vector3.UnitX * 10,
            Vector3.Zero, 4);
        Assert.That(hits.Select(h => h.Source.Entity), Is.EqualTo(new[] { new EntityUid(2), new EntityUid(4), new EntityUid(8) }));
        Assert.That(hits.Select(h => h.Fraction), Is.EqualTo(new[] { 0.15f, 0.35f, 0.75f }).Within(1e-5));
        Assert.That(backend.ShapeCast(new SpatialSphere(0.25f), RigidTransform3.Identity, Vector3.UnitX * 10,
            Vector3.Zero, 2), Is.Empty);
    }

    [Test]
    public void InvalidQueryFailsBeforeTouchingTheBackend()
    {
        using var backend = new BepuSpatialPhysicsBackend(Vector3.Zero);
        Assert.Throws<ArgumentException>(() => backend.ShapeCast(new SpatialSphere(1), default,
            Vector3.UnitX, Vector3.Zero, 1));
        Assert.Throws<ArgumentException>(() => backend.ShapeCast(new SpatialBox(new Vector3(-1)), RigidTransform3.Identity,
            Vector3.UnitX, Vector3.Zero, 1));
        Assert.Throws<ArgumentException>(() => backend.ShapeCast(new SpatialSphere(1), RigidTransform3.Identity,
            new Vector3(float.NaN), Vector3.Zero, 1));
        Assert.Throws<ArgumentException>(() => backend.ShapeCast(new SpatialSphere(1), RigidTransform3.Identity,
            Vector3.Zero, new Vector3(float.MaxValue), 1));
        Assert.That(backend.BodyCount, Is.Zero);
    }
}
