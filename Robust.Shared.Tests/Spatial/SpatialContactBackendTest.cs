using System.Numerics;
using NUnit.Framework;
using Robust.Shared.GameObjects;
using Robust.Shared.Maths;
using Robust.Shared.Spatial;
using Robust.Shared.Spatial.Physics;

namespace Robust.Shared.Tests.Spatial;

[TestFixture]
public sealed class SpatialContactBackendTest
{
    [TestCase(30)]
    [TestCase(60)]
    public void FastBodyProducesOneTouchAgainstThinGeometry(int tickRate)
    {
        using var backend = new BepuSpatialPhysicsBackend(Vector3.Zero);
        backend.AddBody(new SpatialBackendBodyDefinition(SpatialBackendMotion.Static,
            new[] { new SpatialBackendPart(new SpatialBox(new Vector3(0.02f, 5, 5)), RigidTransform3.Identity) }),
            new SpatialBackendState(new RigidTransform3(new Vector3(2, 0, 0), Quaternion.Identity), default, default));
        var body = backend.AddBody(new SpatialBackendBodyDefinition(SpatialBackendMotion.Dynamic,
            new[] { new SpatialBackendPart(new SpatialSphere(0.1f), RigidTransform3.Identity) }),
            new SpatialBackendState(RigidTransform3.Identity, new Vector3(300, 0, 0), default));
        for (var i = 0; i < tickRate; i++)
            backend.Step(1f / tickRate);
        var events = backend.DrainContactEvents();
        Assert.That(events.Count(e => e.Started), Is.EqualTo(1));
        Assert.That(backend.GetState(body).Pose.Position.X, Is.LessThan(1.91));
    }

    [Test]
    public void CompoundContactWitnessUsesChildOffsetAndPreservesSource()
    {
        using var backend = new BepuSpatialPhysicsBackend(Vector3.Zero);
        var source = new EntityUid(4);
        var surface = new SpatialSurfaceReference(source, new SurfaceId(8));
        backend.AddBody(new SpatialBackendBodyDefinition(SpatialBackendMotion.Static,
            new[] { new SpatialBackendPart(new SpatialBox(new Vector3(4, 4, 0.2f)),
                new RigidTransform3(new Vector3(10, 0, -0.1f), Quaternion.Identity), source, surface, 17) }),
            new SpatialBackendState(new RigidTransform3(new Vector3(0, 20, 3),
                Quaternion.CreateFromAxisAngle(Vector3.UnitZ, MathF.PI / 2)), default, default));
        var body = backend.AddBody(new SpatialBackendBodyDefinition(SpatialBackendMotion.Dynamic,
            new[] { new SpatialBackendPart(new SpatialSphere(0.5f), RigidTransform3.Identity, new EntityUid(5)) }),
            new SpatialBackendState(new RigidTransform3(new Vector3(0, 30, 3.45f), Quaternion.Identity), -Vector3.UnitZ, default));
        backend.Step(1f / 60);
        var started = backend.DrainContactEvents();
        Assert.That(started, Has.Count.EqualTo(1));
        Assert.That(started[0].Started, Is.True);
        var contact = started[0].Contact;
        Assert.That(contact.SourceA.Surface, Is.EqualTo(surface));
        Assert.That(contact.SourceA.Revision, Is.EqualTo(17));
        Assert.That(contact.SourceB.Entity, Is.EqualTo(new EntityUid(5)));
        Assert.That(Vector2.Distance(new Vector2(contact.Point.X, contact.Point.Y), new Vector2(0, 30)), Is.LessThan(0.001));
        Assert.That(contact.Point.Z, Is.InRange(2.94f, 3.06f));
        Assert.That(contact.Normal.Z, Is.LessThan(-0.99));
        Assert.That(Vector3.Dot(contact.RelativeVelocity, contact.Normal), Is.LessThan(-0.9));
        backend.SetState(body, new SpatialBackendState(new RigidTransform3(new Vector3(0, 30, 8), Quaternion.Identity), default, default));
        backend.Step(1f / 60);
        var ended = backend.DrainContactEvents();
        Assert.That(ended, Has.Count.EqualTo(1));
        Assert.That(ended[0].Started, Is.False);
        Assert.That(ended[0].Contact.SourceA.Surface, Is.EqualTo(surface));
    }

    [Test]
    public void SleepingAndPauseDoNotCreateFalseContactTransitionsAndDeletionEndsOnce()
    {
        using var backend = new BepuSpatialPhysicsBackend(new Vector3(0, 0, -9.81f));
        var floor = backend.AddBody(new SpatialBackendBodyDefinition(SpatialBackendMotion.Static,
            new[] { new SpatialBackendPart(new SpatialBox(new Vector3(10, 10, 0.2f)), RigidTransform3.Identity) }),
            new SpatialBackendState(new RigidTransform3(new Vector3(0, 0, -0.1f), Quaternion.Identity), default, default));
        var body = backend.AddBody(new SpatialBackendBodyDefinition(SpatialBackendMotion.Dynamic,
            new[] { new SpatialBackendPart(new SpatialSphere(0.5f), RigidTransform3.Identity) }),
            new SpatialBackendState(new RigidTransform3(new Vector3(0, 0, 0.5f), Quaternion.Identity), default, default));
        for (var i = 0; i < 300; i++)
            backend.Step(1f / 60);
        Assert.That(backend.DrainContactEvents().Any(e => e.Started), Is.True);
        for (var i = 0; i < 120; i++)
            backend.Step(1f / 60);
        Assert.That(backend.DrainContactEvents(), Is.Empty);
        backend.SetPaused(body, true);
        for (var i = 0; i < 30; i++)
            backend.Step(1f / 60);
        Assert.That(backend.DrainContactEvents(), Is.Empty);
        backend.SetPaused(body, false);
        backend.Step(1f / 60);
        Assert.That(backend.DrainContactEvents(), Is.Empty);
        backend.RemoveBody(floor);
        var ended = backend.DrainContactEvents();
        Assert.That(ended, Has.Count.EqualTo(1));
        Assert.That(ended[0].Started, Is.False);
        Assert.That(backend.DrainContactEvents(), Is.Empty);
    }

    [Test]
    public void SpeculativeNearbyShapesAndFilteredPairsDoNotReportTouch()
    {
        using var backend = new BepuSpatialPhysicsBackend(Vector3.Zero);
        var part = new SpatialBackendPart(new SpatialSphere(0.5f), RigidTransform3.Identity);
        backend.AddBody(new SpatialBackendBodyDefinition(SpatialBackendMotion.Static, new[] { part }),
            new SpatialBackendState(RigidTransform3.Identity, default, default));
        var body = backend.AddBody(new SpatialBackendBodyDefinition(SpatialBackendMotion.Dynamic, new[] { part }),
            new SpatialBackendState(new RigidTransform3(new Vector3(0, 0, 1.1f), Quaternion.Identity), default, default));
        backend.Step(1f / 60);
        Assert.That(backend.DrainContactEvents(), Is.Empty);
        backend.RemoveBody(body);
        backend.AddBody(new SpatialBackendBodyDefinition(SpatialBackendMotion.Dynamic, new[] { part }, collisionMask: 0),
            new SpatialBackendState(new RigidTransform3(new Vector3(0, 0, 0.9f), Quaternion.Identity), default, default));
        backend.Step(1f / 60);
        Assert.That(backend.DrainContactEvents(), Is.Empty);
    }
}
