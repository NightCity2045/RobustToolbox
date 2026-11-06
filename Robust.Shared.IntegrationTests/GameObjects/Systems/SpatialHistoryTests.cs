using System.Numerics;
using Moq;
using NUnit.Framework;
using Robust.Shared.GameObjects;
using Robust.Shared.Maths;
using Robust.Shared.Spatial;
using Robust.Shared.Spatial.Physics;
using Robust.Shared.Timing;
using Robust.UnitTesting.Server;

namespace Robust.UnitTesting.Shared.GameObjects.Systems;

[TestFixture]
public sealed class SpatialHistoryTests
{
    [Test]
    public void HistoricalQueriesUseBodiesFramesAndGeometryFromTheSameTick()
    {
        var sim = RobustServerSimulation.NewSimulation()
            .RegisterEntitySystems(f => f.LoadExtraSystemType<SpatialHistorySystem>()).InitializeInstance();
        var entities = sim.Resolve<IEntityManager>();
        var history = entities.System<SpatialHistorySystem>();
        var physics = entities.System<SharedSpatialPhysicsSystem>();
        var geometry = entities.System<SharedGeometrySystem>();
        var transforms = entities.System<SharedTransformSystem>();
        var world = entities.System<SharedSpatialSystem>().CreateWorld();
        physics.EnablePhysics(world, new SpatialPhysicsSettings(Vector3.Zero, 1, 8));
        var floor = SpatialGeometryTransactionTests.Frame(entities, world);
        var target = SpatialPhysicsTests.SpawnBody(entities, world, new Vector3(1, 0, -2));
        history.Configure(world, new SpatialHistorySettings(8, 1000, 1000));
        Step(1);
        Assert.That(history.TryGetScene(world, new GameTick(1), out var before), Is.True);
        var context = new SpatialQueryContext(world, world, "lag-compensation", uint.MaxValue);
        var ray = new Vector3(1, 0, 5);
        Assert.That(before!.RayCast(context, ray, -Vector3.UnitZ, 10)!.Value.Surface?.OwnerFrame, Is.EqualTo(floor));
        Assert.That(before.TryGetGeometry(floor, out var oldGeometry), Is.True);

        transforms.SetWorldPose(target, new RigidTransform3(new Vector3(5, 0, -2), Quaternion.Identity));
        using var transaction = geometry.PrepareTransaction(new[]
        {
            new SpatialGeometryEdit(geometry.CaptureSnapshot(floor), layers:
                new Dictionary<LayerId, LayerDefinition> { [new LayerId(1)] = new(-4, "Lower floor") })
        });
        geometry.QueueTransaction(transaction);
        Step(2);
        Assert.That(history.TryGetScene(world, new GameTick(2), out var after), Is.True);
        Assert.That(after!.TryGetGeometry(floor, out var newGeometry), Is.True);
        Assert.That(oldGeometry!.Revision, Is.Zero);
        Assert.That(newGeometry!.Revision, Is.EqualTo(1));
        Assert.That(before.FindSurface(context, ray, -Vector3.UnitZ, 10)!.Value.Point!.Value.Z, Is.EqualTo(0).Within(1e-5));
        Assert.That(after.FindSurface(context, ray, -Vector3.UnitZ, 10)!.Value.Point!.Value.Z, Is.EqualTo(-4).Within(1e-5));
        var throughFloor = new SpatialQueryContext(world, world, "lag-compensation", uint.MaxValue, ignoredEntity: floor);
        Assert.That(before.RayCast(throughFloor, ray, -Vector3.UnitZ, 10)!.Value.Entity, Is.EqualTo(target));
        Assert.That(after.RayCast(throughFloor, ray, -Vector3.UnitZ, 10), Is.Null);
        Assert.That(after.RayCast(context, new Vector3(5, 0, 5), -Vector3.UnitZ, 10)!.Value.Entity, Is.EqualTo(target));
        Assert.That(before.TryGetBodyState(target, out var oldBody), Is.True);
        Assert.That(oldBody.Pose.Position.X, Is.EqualTo(1));
        Assert.That(transforms.GetWorldPose(target).Position.X, Is.EqualTo(5), "Historical queries cannot rewind canonical state.");
        var sphere = new SpatialSphereShape(0.1f);
        Assert.That(before.IsVolumeFree(context, sphere, new RigidTransform3(new Vector3(1, 0, -2), Quaternion.Identity)), Is.False);
        Assert.That(after.IsVolumeFree(context, sphere, new RigidTransform3(new Vector3(1, 0, -2), Quaternion.Identity)), Is.True);
        Assert.That(before.OverlapContacts(context, sphere, new RigidTransform3(new Vector3(1, 0, 0.05f), Quaternion.Identity)), Is.Not.Empty);
        Assert.Throws<InvalidOperationException>(() => before.FindSurface(
            new SpatialQueryContext(world, world, "lag-compensation", uint.MaxValue,
                expectedGeometryRevisions: new Dictionary<EntityUid, ulong> { [floor] = 1 }), ray, -Vector3.UnitZ, 10));

        transforms.SetWorldPose(floor, new RigidTransform3(new Vector3(20, 0, 0), Quaternion.Identity));
        var historicalFrame = new SpatialQueryContext(world, floor, "lag-compensation", uint.MaxValue);
        Assert.That(before.FindSurface(historicalFrame, ray, -Vector3.UnitZ, 10)!.Value.Point!.Value.X, Is.EqualTo(1).Within(1e-5),
            "Input coordinates use the captured frame transform, not its current pose.");
        var recursive = new SpatialQueryContext(world, world, "lag-compensation", uint.MaxValue,
            filter: new CallbackFilter(() => after.RayCast(context, ray, -Vector3.UnitZ, 10)));
        Assert.Throws<InvalidOperationException>(() => before.RayCast(recursive, ray, -Vector3.UnitZ, 10));
        Assert.That(before.RayCast(context, ray, -Vector3.UnitZ, 10), Is.Not.Null,
            "A rejected policy recursion must release the query guard.");

        entities.DeleteEntity(target);
        Step(3);
        Assert.That(before.RayCast(throughFloor, ray, -Vector3.UnitZ, 10)!.Value.Entity, Is.EqualTo(target),
            "Historical identity is retained; content decides whether a deleted target can receive effects.");
        Assert.That(history.TryGetScene(world, new GameTick(3), out var third), Is.True);
        Assert.That(third!.TryGetGeometry(floor, out var unchanged), Is.True);
        Assert.That(unchanged, Is.SameAs(newGeometry), "Unchanged immutable geometry is shared across ticks.");
        Assert.That(history.TryGetScene(world, new GameTick(99), out _), Is.False);
        entities.DeleteEntity(world);
        Assert.That(before.IsAvailable, Is.False);
        Assert.That(history.TryGetScene(world, new GameTick(2), out _), Is.False);
        Assert.Throws<InvalidOperationException>(() => before.RayCast(context, ray, -Vector3.UnitZ, 10));

        void Step(uint tick)
        {
            Mock.Get(sim.Resolve<IGameTiming>()).SetupGet(t => t.CurTick).Returns(new GameTick(tick));
            physics.Update(1f / 60);
        }
    }

    [Test]
    public void WindowSampleBudgetsAndReconfigurationLeaveExplicitGaps()
    {
        var sim = RobustServerSimulation.NewSimulation()
            .RegisterEntitySystems(f => f.LoadExtraSystemType<SpatialHistorySystem>()).InitializeInstance();
        var entities = sim.Resolve<IEntityManager>();
        var physics = entities.System<SharedSpatialPhysicsSystem>();
        var history = entities.System<SpatialHistorySystem>();
        var world = entities.System<SharedSpatialSystem>().CreateWorld();
        physics.EnablePhysics(world, new SpatialPhysicsSettings(Vector3.Zero, 1, 8));
        SpatialPhysicsTests.SpawnBody(entities, world, Vector3.Zero);
        Assert.That(history.TryGetScene(world, new GameTick(1), out _), Is.False, "History has no implicit gameplay window.");
        history.Configure(world, new SpatialHistorySettings(2, 10, 10));
        Step(1);
        history.TryGetScene(world, new GameTick(1), out var first);
        Step(2);
        Step(3);
        Assert.That(first!.IsAvailable, Is.False);
        Assert.That(first.TryGetPose(world, out _), Is.False);
        Assert.That(history.TryGetScene(world, new GameTick(1), out _), Is.False);
        Assert.That(history.TryGetScene(world, new GameTick(2), out _), Is.True);
        history.Configure(world, new SpatialHistorySettings(100, 2, 1));
        Step(4);
        history.TryGetScene(world, new GameTick(4), out var fourth);
        Step(5);
        Assert.That(fourth!.IsAvailable, Is.False, "Aggregate sample budget can shorten the configured time window.");
        Assert.That(history.TryGetScene(world, new GameTick(5), out _), Is.True);
        history.Configure(world, new SpatialHistorySettings(100, 1, 1));
        Step(6);
        Assert.That(history.TryGetScene(world, new GameTick(6), out _), Is.False,
            "An incomplete capture cannot be represented as an empty historical scene.");
        history.Disable(world);
        Step(7);
        Assert.That(history.TryGetScene(world, new GameTick(7), out _), Is.False);
        history.Configure(world, new SpatialHistorySettings(10, 100, 100));
        Step(8);
        history.TryGetScene(world, new GameTick(8), out var last);
        entities.RemoveComponent<SpatialPhysicsWorldComponent>(world);
        Assert.That(last!.IsAvailable, Is.False);
        Assert.That(history.TryGetScene(world, new GameTick(8), out _), Is.False);

        void Step(uint tick)
        {
            Mock.Get(sim.Resolve<IGameTiming>()).SetupGet(t => t.CurTick).Returns(new GameTick(tick));
            physics.Update(1f / 60);
        }
    }

    private sealed class CallbackFilter(Action callback) : ISpatialQueryFilter
    {
        public bool Allow(SpatialQueryContext context, SpatialQueryCandidate candidate)
        {
            callback();
            return true;
        }
    }
}
