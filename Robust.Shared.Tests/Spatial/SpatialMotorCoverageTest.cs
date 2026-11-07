using System;
using System.Numerics;
using NUnit.Framework;
using Robust.Shared.Maths;
using Robust.Shared.Spatial.Physics;

namespace Robust.Shared.Tests.Spatial;

[TestFixture]
public sealed class SpatialMotorCoverageTest
{
    private static SpatialBackendBodyId Motor(BepuSpatialPhysicsBackend backend, Vector3 velocity = default)
        => backend.AddBody(new SpatialBackendBodyDefinition(SpatialBackendMotion.Dynamic,
                new[] { new SpatialBackendPart(new SpatialCapsule(0.1f, 0), RigidTransform3.Identity) }, lockRotation: true),
            new SpatialBackendState(RigidTransform3.Identity, velocity, default));

    [Test]
    public void MissingQueryCoverageSticksForTheWholeScopeAndDoesNotInventAHit()
    {
        using var backend = new BepuSpatialPhysicsBackend(Vector3.Zero);
        var motor = Motor(backend);
        var bounds = new Box3(new Vector3(-1), new Vector3(1));
        using (var coverage = backend.BeginMotorCoverage(motor, bounds))
        {
            Assert.That(coverage.Available, Is.True);
            Assert.That(backend.ShapeCast(new SpatialSphere(0.1f), RigidTransform3.Identity,
                Vector3.UnitX * 2, default, 1, motor), Is.Empty);
            Assert.That(coverage.Available, Is.False);
            backend.OverlapContacts(new SpatialSphere(0.1f), RigidTransform3.Identity, 1, motor);
            Assert.That(coverage.Available, Is.False);
            Assert.Throws<InvalidOperationException>(() => backend.BeginMotorCoverage(motor, bounds));
        }
        using var next = backend.BeginMotorCoverage(motor, bounds);
        Assert.That(next.Available, Is.True);
    }

    [TestCase(1)]
    [TestCase(8)]
    public void FastFreeFlightCannotPublishAnExitEvenWithoutAnyContact(int substeps)
    {
        using var backend = new BepuSpatialPhysicsBackend(Vector3.Zero, substeps);
        var motor = Motor(backend, Vector3.UnitX * 1000);
        using var coverage = backend.BeginMotorCoverage(motor, new Box3(new Vector3(-2), new Vector3(2)));
        backend.Step(1f / 60);
        Assert.That(coverage.Available, Is.False);
    }

    [Test]
    public void ConstraintExcursionCannotBeHiddenByReturningBeforeTheEndOfTheTick()
    {
        using var backend = new BepuSpatialPhysicsBackend(Vector3.Zero, 32);
        var motor = Motor(backend, Vector3.UnitX * 20);
        var anchor = backend.AddBody(new SpatialBackendBodyDefinition(SpatialBackendMotion.Static,
            new[] { new SpatialBackendPart(new SpatialSphere(0.1f), RigidTransform3.Identity) }),
            new SpatialBackendState(RigidTransform3.Identity, default, default));
        backend.AddConstraint(new SpatialBackendConstraintDefinition(SpatialConstraintKind.Point,
            motor, anchor, default, default, RigidTransform3.Identity, SpringFrequency: 10, Damping: 0));
        var bounds = new Box3(new Vector3(-0.3f), new Vector3(0.3f));
        using var coverage = backend.BeginMotorCoverage(motor, bounds);
        Assert.That(coverage.Available, Is.True);
        backend.Step(0.1f);
        var end = backend.GetState(motor).Pose.Position;
        Assert.That(MathF.Abs(end.X), Is.LessThan(0.2f), "The final capsule fits; checking only its final pose misses the excursion.");
        Assert.That(coverage.Available, Is.False);
    }
}
