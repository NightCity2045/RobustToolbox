using System.Numerics;
using NUnit.Framework;
using Moq;
using Robust.Shared.Timing;
using Robust.Shared.GameStates;
using Robust.Shared.GameObjects;
using Robust.Shared.Map;
using Robust.Shared.Maths;
using Robust.UnitTesting.Server;

namespace Robust.UnitTesting.Shared.GameObjects.Systems;

[TestFixture]
public sealed class TransformValidationTests
{
    [TestCase(false, false)]
    [TestCase(false, true)]
    [TestCase(true, false)]
    [TestCase(true, true)]
    public void RejectedCyclePreservesHierarchyAndAnchor(bool anchored, bool self)
    {
        var sim = RobustServerSimulation.NewSimulation().InitializeInstance();
        var entities = sim.Resolve<IEntityManager>();
        var maps = entities.System<SharedMapSystem>();
        var transforms = entities.System<SharedTransformSystem>();
        maps.CreateMap(out var mapId);
        var grid = maps.CreateGridEntity(mapId);
        maps.SetTile(grid, Vector2i.Zero, new Tile(1));
        var parent = entities.SpawnAttachedTo(null, new EntityCoordinates(grid, new Vector2(0.5f)));
        var child = entities.SpawnAttachedTo(null, new EntityCoordinates(parent, Vector2.One));
        var grandchild = entities.SpawnAttachedTo(null, new EntityCoordinates(child, Vector2.One));
        var transform = entities.GetComponent<TransformComponent>(parent);
        transforms.SetLocalRotation(parent, new Angle(0.25));
        if (anchored)
            Assert.That(transforms.TryAnchor((parent, transform), true), Is.True);

        var coordinates = transform.Coordinates;
        var rotation = transform.LocalRotation;
        var matrix = transform.LocalMatrix;
        var modified = transform.LastModifiedFields.ToArray();
        Mock.Get(sim.Resolve<IGameTiming>()).SetupGet(t => t.CurTick).Returns(new GameTick(2));
        var events = 0;
        transforms.OnGlobalMoveEvent += (ref MoveEvent _) => events++;

        Assert.Throws<InvalidOperationException>(() => transforms.SetCoordinates(
            parent, transform, new EntityCoordinates(self ? parent : grandchild, new Vector2(12, -7)), new Angle(1)));

        Assert.Multiple(() =>
        {
            Assert.That(transform.Coordinates, Is.EqualTo(coordinates));
            Assert.That(transform.LocalRotation, Is.EqualTo(rotation));
            Assert.That(transform.LocalMatrix, Is.EqualTo(matrix));
            Assert.That(transform.LastModifiedFields, Is.EqualTo(modified));
            Assert.That(transform.Anchored, Is.EqualTo(anchored));
            Assert.That(transform.ChildCount, Is.EqualTo(1));
            Assert.That(entities.GetComponent<TransformComponent>(child).ParentUid, Is.EqualTo(parent));
            Assert.That(entities.GetComponent<TransformComponent>(grandchild).ParentUid, Is.EqualTo(child));
            Assert.That(entities.IsQueuedForDeletion(parent), Is.False);
            Assert.That(maps.IsAnchored(grid, grid.Comp, Vector2i.Zero, parent), Is.EqualTo(anchored));
            Assert.That(events, Is.Zero);
        });
    }

    [TestCase(float.NaN)]
    [TestCase(float.PositiveInfinity)]
    [TestCase(float.NegativeInfinity)]
    public void RejectedNumbersPreservePose(float invalid)
    {
        var sim = RobustServerSimulation.NewSimulation().InitializeInstance();
        var entities = sim.Resolve<IEntityManager>();
        var map = sim.CreateMap().MapId;
        var uid = entities.Spawn(null, new MapCoordinates(new Vector2(2, 3), map));
        var transform = entities.GetComponent<TransformComponent>(uid);
        var transforms = entities.System<SharedTransformSystem>();
        var coordinates = transform.Coordinates;
        var modified = transform.LastModifiedFields.ToArray();
        Mock.Get(sim.Resolve<IGameTiming>()).SetupGet(t => t.CurTick).Returns(new GameTick(2));
        var events = 0;
        transforms.OnGlobalMoveEvent += (ref MoveEvent _) => events++;

        Action[] writes =
        [
            () => transforms.SetLocalPosition(uid, new Vector2(invalid, 1)),
            () => transforms.SetLocalRotation(uid, new Angle(invalid)),
            () => transforms.SetLocalPositionRotation(uid, new Vector2(4, 5), new Angle(invalid)),
            () => transforms.SetLocalPositionRotation(uid, new Vector2(1, invalid), new Angle(1)),
            () => transforms.SetCoordinates(uid, transform, new EntityCoordinates(transform.ParentUid, new Vector2(4, 5)), new Angle(invalid)),
            () => transforms.SetCoordinates(uid, transform, new EntityCoordinates(transform.ParentUid, new Vector2(invalid, 1)), new Angle(1)),
        ];

        foreach (var write in writes)
        {
            Assert.Throws<ArgumentOutOfRangeException>(() => write());
            Assert.Multiple(() =>
            {
                Assert.That(transform.Coordinates, Is.EqualTo(coordinates));
                Assert.That(transform.LocalRotation, Is.EqualTo(Angle.Zero));
                Assert.That(transform.LastModifiedFields, Is.EqualTo(modified));
                Assert.That(events, Is.Zero);
            });
        }
    }

    [Test]
    public void AuthoritativeParentSwapBreaksTemporaryPredictionCycle()
    {
        var sim = RobustServerSimulation.NewSimulation().InitializeInstance();
        var entities = sim.Resolve<IEntityManager>();
        var map = sim.CreateMap().MapId;
        var transforms = entities.System<SharedTransformSystem>();
        var parent = entities.Spawn(null, new MapCoordinates(Vector2.One, map));
        var child = entities.SpawnAttachedTo(null, new EntityCoordinates(parent, new Vector2(2, 0)));
        var parentTransform = entities.GetComponent<TransformComponent>(parent);
        var childTransform = entities.GetComponent<TransformComponent>(child);
        var mapEntity = parentTransform.ParentUid;
        Mock.Get(sim.Resolve<IGameTiming>()).SetupGet(t => t.ApplyingState).Returns(true);

        // The authoritative order is child -> parent. Its first state temporarily
        // conflicts with the predicted order parent -> child.
        var parentState = new ComponentHandleState(new TransformComponentState(
            new Vector2(1, 0), Angle.Zero, entities.GetNetEntity(child), false, false), null);
        transforms.OnHandleState(parent, parentTransform, ref parentState);
        Assert.That(childTransform.ParentUid, Is.EqualTo(EntityUid.Invalid));
        Assert.That(parentTransform.ParentUid, Is.EqualTo(child));

        var childState = new ComponentHandleState(new TransformComponentState(
            new Vector2(3, 4), Angle.Zero, entities.GetNetEntity(mapEntity), false, false), null);
        transforms.OnHandleState(child, childTransform, ref childState);
        Assert.Multiple(() =>
        {
            Assert.That(transforms.GetWorldPosition(parent), Is.EqualTo(new Vector2(4, 4)));
            Assert.That(transforms.GetWorldPosition(child), Is.EqualTo(new Vector2(3, 4)));
            Assert.That(parentTransform.ChildCount, Is.Zero);
            Assert.That(childTransform.ChildCount, Is.EqualTo(1));
            Assert.That(entities.IsQueuedForDeletion(parent), Is.False);
            Assert.That(entities.IsQueuedForDeletion(child), Is.False);
        });
    }


    [TestCase(float.NaN)]
    [TestCase(float.PositiveInfinity)]
    [TestCase(float.NegativeInfinity)]
    public void RejectedSpawnDoesNotAllocateEntity(float invalid)
    {
        var sim = RobustServerSimulation.NewSimulation().InitializeInstance();
        var entities = sim.Resolve<IEntityManager>();
        var maps = entities.System<SharedMapSystem>();
        var mapEntity = maps.CreateMap(out var mapId);
        var count = entities.EntityCount;
        Assert.Throws<ArgumentOutOfRangeException>(() => entities.CreateEntityUninitialized(
            null, new MapCoordinates(new Vector2(invalid, 0), mapId)));
        Assert.Throws<ArgumentOutOfRangeException>(() => entities.CreateEntityUninitialized(
            null, new EntityCoordinates(mapEntity, new Vector2(0, invalid))));
        Assert.Throws<ArgumentOutOfRangeException>(() => entities.CreateEntityUninitialized(
            null, new MapCoordinates(Vector2.Zero, mapId), rotation: new Angle(invalid)));
        Assert.Throws<ArgumentOutOfRangeException>(() => entities.CreateEntityUninitialized(
            null, new EntityCoordinates(mapEntity, Vector2.Zero), rotation: new Angle(invalid)));
        Assert.That(entities.EntityCount, Is.EqualTo(count));
    }

}
