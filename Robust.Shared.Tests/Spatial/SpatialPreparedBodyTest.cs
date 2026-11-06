using System.Numerics;
using NUnit.Framework;
using Robust.Shared.Maths;
using Robust.Shared.Spatial.Physics;

namespace Robust.Shared.Tests.Spatial;

[TestFixture]
public sealed class SpatialPreparedBodyTest
{
    private static SpatialBackendBodyDefinition Definition() => new(SpatialBackendMotion.Dynamic,
        new[] { new SpatialBackendPart(new SpatialSphere(0.5f), RigidTransform3.Identity) });
    private static SpatialBackendState State() => new(RigidTransform3.Identity, default, default);

    [Test]
    public void PreparedShapesAreAbsentFromSceneUntilOneSuccessfulCommit()
    {
        using var backend = new BepuSpatialPhysicsBackend(Vector3.Zero);
        using var prepared = backend.PrepareBody(Definition());
        Assert.That(backend.BodyCount, Is.Zero);
        Assert.That(backend.RayCast(Vector3.UnitZ * 2, -Vector3.UnitZ, 4, uint.MaxValue), Is.Null);
        Assert.That(backend.Overlap(new SpatialSphere(1), RigidTransform3.Identity, uint.MaxValue), Is.Empty);
        backend.Step(1f / 30);
        Assert.That(backend.DrainContactEvents(), Is.Empty);
        Assert.Throws<ArgumentException>(() => prepared.Commit(new SpatialBackendState(default, default, default)));
        Assert.That(backend.BodyCount, Is.Zero);
        var id = prepared.Commit(State());
        Assert.That(backend.BodyCount, Is.EqualTo(1));
        Assert.That(backend.RayCast(Vector3.UnitZ * 2, -Vector3.UnitZ, 4, uint.MaxValue)!.Value.Body, Is.EqualTo(id));
        Assert.Throws<InvalidOperationException>(() => prepared.Commit(State()));
        prepared.Dispose();
        Assert.That(backend.GetState(id), Is.EqualTo(State()));
    }

    [Test]
    public void AbandonedPreparedBodiesReleaseTheirShapesWithoutCreatingEntities()
    {
        using var backend = new BepuSpatialPhysicsBackend(Vector3.Zero);
        for (var i = 0; i < 100; i++)
        {
            using var prepared = backend.PrepareBody(Definition());
            prepared.ValidateState(State());
        }
        Assert.That(backend.BodyCount, Is.Zero);
        var id = backend.AddBody(Definition(), State());
        backend.Step(1f / 30);
        Assert.That(backend.GetState(id), Is.EqualTo(State()));
        backend.RemoveBody(id);
        Assert.That(backend.DrainContactEvents(), Is.Empty);
    }

    [Test]
    public void OwnerDisposalInvalidatesPreparedBodiesWithoutDoubleFree()
    {
        var backend = new BepuSpatialPhysicsBackend(Vector3.Zero);
        var prepared = backend.PrepareBody(Definition());
        backend.Dispose();
        Assert.Throws<ObjectDisposedException>(() => prepared.Commit(State()));
        Assert.DoesNotThrow(prepared.Dispose);
        Assert.DoesNotThrow(prepared.Dispose);
    }
}
