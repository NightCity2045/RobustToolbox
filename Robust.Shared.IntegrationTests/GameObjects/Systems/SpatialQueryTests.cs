using System.Numerics;
using NUnit.Framework;
using Robust.Shared.GameObjects;
using Robust.Shared.Map.Components;
using Robust.Shared.Maths;
using Robust.Shared.Spatial;
using Robust.Shared.Spatial.Physics;
using Robust.UnitTesting.Server;

namespace Robust.UnitTesting.Shared.GameObjects.Systems;

[TestFixture]
public sealed class SpatialQueryTests
{
    [Test]
    public void QueriesResolveInputFrameAndPreserveSurfaceMaterialAndRevision()
    {
        var sim = RobustServerSimulation.NewSimulation().InitializeInstance();
        var entities = sim.Resolve<IEntityManager>();
        var spatial = entities.System<SharedSpatialSystem>();
        var world = spatial.CreateWorld();
        entities.System<SharedSpatialPhysicsSystem>().EnablePhysics(world);
        var frame = SpatialGeometryTests.CreateUninitializedFrame(entities, world);
        entities.System<SharedTransformSystem>().SetLocalPose(frame, new RigidTransform3(new Vector3(10, 20, 3),
            Quaternion.CreateFromAxisAngle(Vector3.UnitZ, MathF.PI / 2)));
        var surface = new SpatialSurfaceReference(frame, new SurfaceId(8));
        var slab = new SlabSurface(surface, new LayerId(5), new Box2(-5, -5, 5, 5), 0, 0.2f,
            materialProfile: "test-floor", stateRevision: 4);
        entities.System<SharedGeometrySystem>().ConfigureInitialGeometry(
            (frame, entities.GetComponent<SpatialGeometryComponent>(frame)), Array.Empty<BoundarySegment>(), new[] { slab }, 4);
        entities.InitializeAndStartEntity(frame, entities.GetComponent<MapComponent>(world).MapId);
        var body = SpatialPhysicsTests.SpawnBody(entities, frame, new Vector3(0, 0, 3));
        ISpatialQueryService queries = entities.System<SpatialQuerySystem>();
        var revisions = new Dictionary<EntityUid, ulong> { [frame] = 4 };
        var context = new SpatialQueryContext(world, frame, "interaction", uint.MaxValue, expectedGeometryRevisions: revisions);
        revisions[frame] = 99; // The context owns its detached revision expectations.
        var origin = new Vector3(0, 0, 5);
        var hit = queries.RayCast(context, origin, -Vector3.UnitZ, 10)!.Value;
        Assert.That(hit.Entity, Is.EqualTo(body));
        Assert.That(hit.Distance, Is.EqualTo(1.75f).Within(1e-4));
        var floor = queries.FindSurface(context, origin, -Vector3.UnitZ, 10)!.Value;
        Assert.That(floor.Surface, Is.EqualTo(surface));
        Assert.That(floor.MaterialProfile, Is.EqualTo("test-floor"));
        Assert.That(floor.Revision, Is.EqualTo(4));
        Assert.That(Vector3.Distance(floor.Point!.Value, new Vector3(10, 20, 4.75f)), Is.LessThan(1e-4));
        Assert.That(floor.Normal!.Value.Z, Is.GreaterThan(0.99));
        var filter = new SurfaceOnlyFilter();
        var filtered = new SpatialQueryContext(world, frame, "placement", uint.MaxValue, filter: filter);
        Assert.That(queries.RayCast(filtered, origin, -Vector3.UnitZ, 10)!.Value.Surface, Is.EqualTo(surface));
        Assert.That(filter.Purposes, Does.Contain("placement"));
        var ignoring = new SpatialQueryContext(world, frame, "collision", uint.MaxValue, ignoredEntity: body);
        Assert.That(queries.RayCast(ignoring, origin, -Vector3.UnitZ, 10)!.Value.Surface, Is.EqualTo(surface));
        var shape = new SpatialSphereShape(0.1f);
        Assert.That(queries.IsVolumeFree(context, shape, new RigidTransform3(new Vector3(3, 0, 5), Quaternion.Identity)), Is.True);
        Assert.That(queries.IsVolumeFree(context, shape, new RigidTransform3(new Vector3(3, 0, 1.75f), Quaternion.Identity)), Is.False);
        var cast = queries.ShapeCast(filtered, shape, new RigidTransform3(new Vector3(3, 0, 5), Quaternion.Identity),
            -Vector3.UnitZ * 5, Vector3.Zero);
        Assert.That(cast, Has.Count.EqualTo(1));
        Assert.That(cast[0].Surface, Is.EqualTo(surface));
        Assert.That(cast[0].Distance, Is.EqualTo(3.15f).Within(0.001));
        Assert.That(cast[0].Fraction, Is.EqualTo(0.63f).Within(0.001));
        entities.DeleteEntity(world);
        Assert.Throws<InvalidOperationException>(() => queries.IsVolumeFree(context, shape, RigidTransform3.Identity));
    }

    [Test]
    public void WrongWorldStaleRevisionAndUnavailableSceneNeverReportFreeSpace()
    {
        var sim = RobustServerSimulation.NewSimulation().InitializeInstance();
        var entities = sim.Resolve<IEntityManager>();
        var spatial = entities.System<SharedSpatialSystem>();
        var first = spatial.CreateWorld();
        var second = spatial.CreateWorld();
        var queries = entities.System<SpatialQuerySystem>();
        var shape = new SpatialBoxShape(Vector3.One);
        Assert.Throws<InvalidOperationException>(() => queries.IsVolumeFree(
            new SpatialQueryContext(first, first, "placement", 1), shape, RigidTransform3.Identity));
        entities.System<SharedSpatialPhysicsSystem>().EnablePhysics(first);
        Assert.Throws<InvalidOperationException>(() => queries.IsVolumeFree(
            new SpatialQueryContext(first, second, "placement", 1), shape, RigidTransform3.Identity));
        Assert.Throws<InvalidOperationException>(() => queries.IsVolumeFree(
            new SpatialQueryContext(first, first, "placement", 1, ignoredEntity: second), shape, RigidTransform3.Identity));
        Assert.Throws<InvalidOperationException>(() => queries.IsVolumeFree(
            new SpatialQueryContext(first, first, "placement", 1,
                expectedGeometryRevisions: new Dictionary<EntityUid, ulong> { [first] = 42 }), shape, RigidTransform3.Identity));
        Assert.That(queries.IsVolumeFree(new SpatialQueryContext(first, first, "placement", 1), shape,
            RigidTransform3.Identity), Is.True);
    }

    private sealed class SurfaceOnlyFilter : ISpatialQueryFilter
    {
        public readonly HashSet<string> Purposes = new();
        public bool Allow(SpatialQueryContext context, SpatialQueryCandidate candidate)
        {
            Purposes.Add(context.Purpose);
            return candidate.Surface != null;
        }
    }
}
