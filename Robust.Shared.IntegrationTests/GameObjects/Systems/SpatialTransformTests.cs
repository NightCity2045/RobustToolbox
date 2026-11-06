using System.Numerics;
using NUnit.Framework;
using Robust.Shared.GameObjects;
using Robust.Shared.GameStates;
using Robust.Shared.Containers;
using Robust.Shared.Map;
using Robust.Shared.Maths;
using Robust.UnitTesting.Server;

namespace Robust.UnitTesting.Shared.GameObjects.Systems;

[TestFixture]
public sealed class SpatialTransformTests
{
    [Test]
    public void NestedFramesAndReparentKeepCompleteWorldPose()
    {
        var sim = RobustServerSimulation.NewSimulation().InitializeInstance();
        var entities = sim.Resolve<IEntityManager>();
        var map = sim.CreateMap().MapId;
        var transforms = entities.System<SharedTransformSystem>();
        var parent = entities.Spawn(null, new MapCoordinates(Vector2.Zero, map));
        var target = entities.Spawn(null, new MapCoordinates(Vector2.Zero, map));
        var child = entities.SpawnAttachedTo(null, new EntityCoordinates(parent, Vector2.Zero));
        transforms.SetLocalPose(parent, new RigidTransform3(new Vector3(10, -2, 5),
            Quaternion.CreateFromAxisAngle(Vector3.UnitZ, MathF.PI / 2)));
        transforms.SetLocalPose(child, new RigidTransform3(new Vector3(2, 0, 3),
            Quaternion.CreateFromAxisAngle(Vector3.UnitX, MathF.PI / 2)));
        transforms.SetLocalPose(target, new RigidTransform3(new Vector3(-100, 40, -9),
            Quaternion.CreateFromYawPitchRoll(0.3f, -1.2f, 0.8f)));
        var world = transforms.GetWorldPose(child);
        Near(world.Position, new Vector3(10, 0, 8));
        Near(world.TransformVector(Vector3.UnitY), Vector3.UnitZ);

        transforms.SetParent(child, target);
        SamePose(transforms.GetWorldPose(child), world);
        Assert.That(entities.GetComponent<TransformComponent>(child).ParentUid, Is.EqualTo(target));
        Assert.That(entities.GetComponent<TransformComponent>(parent).ChildCount, Is.Zero);
        Assert.That(entities.GetComponent<TransformComponent>(target).ChildCount, Is.EqualTo(1));

        var localPoint = new Vector3(4, -3, 2);
        var converted = transforms.ConvertPoint(child, parent, localPoint);
        Near(transforms.GetWorldPose(parent).TransformPoint(converted), world.TransformPoint(localPoint));
    }

    [Test]
    public void PlanarWritesPreserveHeightAndTiltAndMovesReportHeight()
    {
        var sim = RobustServerSimulation.NewSimulation().InitializeInstance();
        var entities = sim.Resolve<IEntityManager>();
        var map = sim.CreateMap().MapId;
        var transforms = entities.System<SharedTransformSystem>();
        var uid = entities.Spawn(null, new MapCoordinates(Vector2.Zero, map));
        var transform = entities.GetComponent<TransformComponent>(uid);
        transforms.SetLocalPose(uid, new RigidTransform3(new Vector3(2, 3, -7.5f),
            Quaternion.CreateFromAxisAngle(Vector3.UnitX, MathF.PI / 4)));
        transforms.SetLocalPosition(uid, new Vector2(-10, 40));
        transforms.SetLocalPositionRotation(uid, new Vector2(20, -30), Angle.FromDegrees(90));
        transforms.SetCoordinates(uid, new EntityCoordinates(transform.ParentUid, new Vector2(-5, 6)));
        var pose = transform.LocalPose;
        Near(pose.Position, new Vector3(-5, 6, -7.5f));
        Near(pose.TransformVector(Vector3.UnitZ), new Vector3(MathF.Sqrt(0.5f), 0, MathF.Sqrt(0.5f)));

        MoveEvent? moved = null;
        transforms.OnGlobalMoveEvent += (ref MoveEvent ev) => { if (ev.Sender == uid) moved = ev; };
        transforms.SetLocalPose(uid, new RigidTransform3(new Vector3(-5, 6, 0.25f), pose.Rotation));
        Assert.That(moved.HasValue, Is.True);
        Assert.Multiple(() =>
        {
            Assert.That(moved!.Value.OnlyRotation, Is.False);
            Assert.That(moved.Value.ParentChanged, Is.False);
            Assert.That(moved.Value.OldPose.Position.Z, Is.EqualTo(-7.5f));
            Assert.That(moved.Value.NewPose.Position.Z, Is.EqualTo(0.25f));
            Assert.That(transform.ParentUid, Is.EqualTo(moved.Value.OldPosition.EntityId));
        });
    }

    [Test]
    public void FullAndDeltaCorrectionRestoreHeightOrientationAndNoRotationTransition()
    {
        var sim = RobustServerSimulation.NewSimulation().InitializeInstance();
        var entities = sim.Resolve<IEntityManager>();
        var map = sim.CreateMap().MapId;
        var transforms = entities.System<SharedTransformSystem>();
        var uid = entities.Spawn(null, new MapCoordinates(Vector2.Zero, map));
        var transform = entities.GetComponent<TransformComponent>(uid);
        transforms.SetNoLocalRotation((uid, transform), true);
        var pose = new RigidTransform3(new Vector3(-20, 10, 45), Quaternion.CreateFromYawPitchRoll(0.2f, 0.8f, -0.4f));
        var full = new TransformComponentState(pose.Position, pose.Rotation,
            entities.GetNetEntity(transform.ParentUid), false, false);
        var state = new ComponentHandleState(full, null);
        transforms.OnHandleState(uid, transform, ref state);
        SamePose(transform.LocalPose, pose);
        Assert.That(transform.NoLocalRotation, Is.False);

        var delta = new TransformComponentDeltaState
        {
            ChangedFields = 1UL << SharedTransformSystem.TransformLocalPositionIndex,
            LocalPosition = new Vector3(-20, 10, -0.5f),
        };
        state = new ComponentHandleState(delta, full);
        transforms.OnHandleState(uid, transform, ref state);
        SamePose(transform.LocalPose, new RigidTransform3(delta.LocalPosition, pose.Rotation));
        Assert.That(transform.NextPosition, Is.EqualTo(pose.Position));
        Assert.That(transform.NextRotation, Is.EqualTo(pose.Rotation));
        transforms.SetNoLocalRotation((uid, transform), true);
        Assert.That(transform.LocalPose.Rotation, Is.EqualTo(Quaternion.Identity));
    }

    [Test]
    public void RejectedSpatialReparentDoesNotChangeAnyPoseAxis()
    {
        var sim = RobustServerSimulation.NewSimulation().InitializeInstance();
        var entities = sim.Resolve<IEntityManager>();
        var map = sim.CreateMap().MapId;
        var transforms = entities.System<SharedTransformSystem>();
        var uid = entities.Spawn(null, new MapCoordinates(Vector2.Zero, map));
        var child = entities.SpawnAttachedTo(null, new EntityCoordinates(uid, Vector2.One));
        var pose = new RigidTransform3(new Vector3(-100000, 3, 4000), Quaternion.CreateFromYawPitchRoll(-0.9f, 0.3f, 1.2f));
        transforms.SetLocalPose(uid, pose);
        var transform = entities.GetComponent<TransformComponent>(uid);
        var parent = transform.ParentUid;
        Assert.Throws<InvalidOperationException>(() => transforms.SetParentLocalPose(uid, child, RigidTransform3.Identity));
        Assert.Throws<ArgumentOutOfRangeException>(() => transforms.SetLocalPose(uid, default));
        SamePose(transform.LocalPose, pose);
        Assert.That(transform.ParentUid, Is.EqualTo(parent));
    }

    private static void SamePose(RigidTransform3 actual, RigidTransform3 expected)
    {
        Near(actual.Position, expected.Position);
        foreach (var axis in new[] { Vector3.UnitX, Vector3.UnitY, Vector3.UnitZ })
            Near(actual.TransformVector(axis), expected.TransformVector(axis));
    }

    private static void Near(Vector3 actual, Vector3 expected)
        => Assert.That(Vector3.Distance(actual, expected), Is.LessThan(0.0001f), $"Expected {expected}, got {actual}");

    [Test]
    public void ExplicitTiltedPlaneWritePreservesDepthAndWorldOrientation()
    {
        var sim = RobustServerSimulation.NewSimulation().InitializeInstance();
        var entities = sim.Resolve<IEntityManager>();
        var map = sim.CreateMap().MapId;
        var transforms = entities.System<SharedTransformSystem>();
        var plane = entities.Spawn(null, new MapCoordinates(Vector2.Zero, map));
        var uid = entities.Spawn(null, new MapCoordinates(Vector2.Zero, map));
        transforms.SetLocalPose(plane, new RigidTransform3(new Vector3(0, 10, 0),
            Quaternion.CreateFromAxisAngle(Vector3.UnitX, MathF.PI / 2)));
        var before = new RigidTransform3(new Vector3(3, 4, 5), Quaternion.CreateFromYawPitchRoll(0.7f, -0.3f, 1.2f));
        transforms.SetWorldPose(uid, before);
        Near(transforms.GetPoseRelativeTo(uid, plane).Position, new Vector3(3, 5, 6));

        transforms.SetPlanarPosition(uid, plane, new Vector2(-2, 7));
        var after = transforms.GetWorldPose(uid);
        SamePose(after, new RigidTransform3(new Vector3(-2, 4, 7), before.Rotation));
        Near(transforms.GetPoseRelativeTo(uid, plane).Position, new Vector3(-2, 7, 6));
        Assert.That(Vector2.Distance(transforms.GetPlanarPosition(uid, plane), new Vector2(-2, 7)), Is.LessThan(1e-5));
        Near(Vector3.Transform(Vector3.One, transforms.GetWorldMatrix3(uid)), after.TransformPoint(Vector3.One));
        Near(Vector3.Transform(after.TransformPoint(Vector3.One), transforms.GetInvWorldMatrix3(uid)), Vector3.One);
    }

    [Test]
    public void CrossWorldProjectionAndWriteFailWithoutChangingPose()
    {
        var sim = RobustServerSimulation.NewSimulation().InitializeInstance();
        var entities = sim.Resolve<IEntityManager>();
        var firstMap = sim.CreateMap().MapId;
        var secondMap = sim.CreateMap().MapId;
        var transforms = entities.System<SharedTransformSystem>();
        var uid = entities.Spawn(null, new MapCoordinates(Vector2.One, firstMap));
        var otherFrame = entities.Spawn(null, new MapCoordinates(Vector2.Zero, secondMap));
        var before = transforms.GetWorldPose(uid);
        Assert.Throws<InvalidOperationException>(() => transforms.ConvertPoint(uid, otherFrame, Vector3.One));
        Assert.Throws<InvalidOperationException>(() => transforms.GetPlanarPosition(uid, otherFrame));
        Assert.Throws<InvalidOperationException>(() => transforms.SetPlanarPosition(uid, otherFrame, Vector2.Zero));
        SamePose(transforms.GetWorldPose(uid), before);
    }

    [TestCase(false, false)]
    [TestCase(false, true)]
    [TestCase(true, false)]
    [TestCase(true, true)]
    public void RejectedNetworkParentPreservesAnchorLookup(bool anchored, bool self)
    {
        var sim = RobustServerSimulation.NewSimulation().InitializeInstance();
        var entities = sim.Resolve<IEntityManager>();
        var maps = entities.System<SharedMapSystem>();
        var transforms = entities.System<SharedTransformSystem>();
        maps.CreateMap(out var mapId);
        var grid = maps.CreateGridEntity(mapId);
        maps.SetTile(grid, Vector2i.Zero, new Tile(1));
        var uid = entities.SpawnAttachedTo(null, new EntityCoordinates(grid, new Vector2(0.5f)));
        var child = entities.SpawnAttachedTo(null, new EntityCoordinates(uid, Vector2.One));
        var transform = entities.GetComponent<TransformComponent>(uid);
        transforms.TryAnchor((uid, transform), anchored);
        var before = transform.LocalPose;
        var state = new ComponentHandleState(new TransformComponentState(
            new Vector3(7, 8, 9), Quaternion.Identity, entities.GetNetEntity(self ? uid : child), false, !anchored), null);
        Assert.Throws<InvalidOperationException>(() => transforms.OnHandleState(uid, transform, ref state));
        Assert.Multiple(() =>
        {
            Assert.That(transform.Anchored, Is.EqualTo(anchored));
            Assert.That(maps.IsAnchored(grid, Vector2i.Zero, uid), Is.EqualTo(anchored));
            Assert.That(transform.ParentUid, Is.EqualTo(grid.Owner));
            Assert.That(entities.GetComponent<TransformComponent>(child).ParentUid, Is.EqualTo(uid));
            Assert.That(transform.LocalPose, Is.EqualTo(before));
        });
    }

    [TestCase(13)]
    [TestCase(37)]
    [TestCase(123)]
    [TestCase(287)]
    [TestCase(359)]
    public void PlanarYawResetHasExactIdentityOrientation(int degrees)
    {
        var sim = RobustServerSimulation.NewSimulation().InitializeInstance();
        var entities = sim.Resolve<IEntityManager>();
        var map = sim.CreateMap().MapId;
        var transforms = entities.System<SharedTransformSystem>();
        var uid = entities.Spawn(null, new MapCoordinates(Vector2.Zero, map));
        transforms.SetLocalRotation(uid, Angle.FromDegrees(degrees));
        transforms.SetLocalRotation(uid, Angle.Zero);
        Assert.That(entities.GetComponent<TransformComponent>(uid).LocalPose.Rotation, Is.EqualTo(Quaternion.Identity));
    }

    [Test]
    public void ContainerInsertionUsesIdentityPoseAndRemovalPreservesWorldPose()
    {
        var sim = RobustServerSimulation.NewSimulation().InitializeInstance();
        var entities = sim.Resolve<IEntityManager>();
        var map = sim.CreateMap().MapId;
        var transforms = entities.System<SharedTransformSystem>();
        var containers = entities.System<SharedContainerSystem>();
        var owner = entities.Spawn(null, new MapCoordinates(Vector2.Zero, map));
        var item = entities.Spawn(null, new MapCoordinates(Vector2.Zero, map));
        var ownerPose = new RigidTransform3(new Vector3(-3, 4, 7), Quaternion.CreateFromYawPitchRoll(0.4f, -0.7f, 0.3f));
        transforms.SetLocalPose(owner, ownerPose);
        transforms.SetLocalPose(item, new RigidTransform3(new Vector3(1, -8, 13), Quaternion.CreateFromYawPitchRoll(-0.3f, 0.2f, 0.8f)));
        var container = containers.EnsureContainer<Container>(owner, "spatial");
        Assert.That(containers.Insert(item, container), Is.True);
        var transform = entities.GetComponent<TransformComponent>(item);
        Assert.That(transform.LocalPose, Is.EqualTo(RigidTransform3.Identity));
        Assert.That(transform.ParentUid, Is.EqualTo(owner));
        SamePose(transforms.GetWorldPose(item), ownerPose);
        Assert.That(containers.Remove(item, container), Is.True);
        SamePose(transforms.GetWorldPose(item), ownerPose);
        Assert.That(container.ContainedEntities, Is.Empty);
    }

    [Test]
    public void MovingFrameBetweenMapsTransfersDescendantsAndDeletionOwnership()
    {
        var sim = RobustServerSimulation.NewSimulation().InitializeInstance();
        var entities = sim.Resolve<IEntityManager>();
        var maps = entities.System<SharedMapSystem>();
        var transforms = entities.System<SharedTransformSystem>();
        var source = maps.CreateMap(out var sourceId);
        var target = maps.CreateMap(out var targetId);
        var frame = entities.Spawn(null, new MapCoordinates(Vector2.Zero, sourceId));
        var child = entities.SpawnAttachedTo(null, new EntityCoordinates(frame, Vector2.One));
        var nested = entities.SpawnAttachedTo(null, new EntityCoordinates(child, -Vector2.One));
        transforms.SetLocalPose(frame, new RigidTransform3(new Vector3(-10, 20, 35),
            Quaternion.CreateFromYawPitchRoll(0.7f, -0.2f, 1.1f)));
        transforms.SetLocalPose(child, new RigidTransform3(new Vector3(2, -3, -4),
            Quaternion.CreateFromYawPitchRoll(-0.5f, 0.8f, 0.3f)));
        var before = new[] { transforms.GetWorldPose(frame), transforms.GetWorldPose(child), transforms.GetWorldPose(nested) };

        transforms.SetParent(frame, target);
        var descendants = new[] { frame, child, nested };
        for (var i = 0; i < descendants.Length; i++)
        {
            var transform = entities.GetComponent<TransformComponent>(descendants[i]);
            Assert.That(transform.MapUid, Is.EqualTo(target));
            Assert.That(transform.MapID, Is.EqualTo(targetId));
            SamePose(transforms.GetWorldPose(descendants[i]), before[i]);
        }
        Assert.That(entities.GetComponent<TransformComponent>(source).ChildCount, Is.Zero);
        maps.DeleteMap(sourceId);
        foreach (var uid in descendants)
            Assert.That(entities.EntityExists(uid), Is.True);

        maps.DeleteMap(targetId);
        foreach (var uid in descendants)
            Assert.That(entities.Deleted(uid), Is.True);
        Assert.That(maps.MapExists(sourceId), Is.False);
        Assert.That(maps.MapExists(targetId), Is.False);
    }

    [Test]
    public void DetachClearsSpatialInterpolationAndParentOwnership()
    {
        var sim = RobustServerSimulation.NewSimulation().InitializeInstance();
        var entities = sim.Resolve<IEntityManager>();
        var map = sim.CreateMap().MapId;
        var transforms = entities.System<SharedTransformSystem>();
        var parent = entities.Spawn(null, new MapCoordinates(Vector2.Zero, map));
        var child = entities.SpawnAttachedTo(null, new EntityCoordinates(parent, Vector2.One));
        var transform = entities.GetComponent<TransformComponent>(child);
        transforms.SetLocalPose(child, new RigidTransform3(new Vector3(3, 4, 5),
            Quaternion.CreateFromYawPitchRoll(0.2f, -0.3f, 0.4f)));
        transform.NextPosition = new Vector3(7, 8, 9);
        transform.NextRotation = Quaternion.CreateFromAxisAngle(Vector3.UnitX, 0.7f);
        transform.LerpParent = parent;

        transforms.DetachEntity(child);
        Assert.Multiple(() =>
        {
            Assert.That(transform.ParentUid, Is.EqualTo(EntityUid.Invalid));
            Assert.That(transform.MapUid, Is.Null);
            Assert.That(transform.LocalPose, Is.EqualTo(RigidTransform3.Identity));
            Assert.That(transform.NextPosition, Is.Null);
            Assert.That(transform.NextRotation, Is.Null);
            Assert.That(transform.LerpParent, Is.EqualTo(EntityUid.Invalid));
            Assert.That(entities.GetComponent<TransformComponent>(parent).ChildCount, Is.Zero);
        });
        entities.DeleteEntity(parent);
        Assert.That(entities.EntityExists(child), Is.True);
        entities.DeleteEntity(child);
    }

    [Test]
    public void SpatialSpawnHasCompletePoseBeforeInitializationCompletes()
    {
        var sim = RobustServerSimulation.NewSimulation().InitializeInstance();
        var entities = sim.Resolve<IEntityManager>();
        var maps = entities.System<SharedMapSystem>();
        var transforms = entities.System<SharedTransformSystem>();
        maps.CreateMap(out var mapId);
        var frame = entities.Spawn(null, new MapCoordinates(Vector2.Zero, mapId));
        var framePose = new RigidTransform3(new Vector3(20, -10, 6),
            Quaternion.CreateFromYawPitchRoll(0.5f, -0.2f, 0.3f));
        var local = new RigidTransform3(new Vector3(-3, 4, 7),
            Quaternion.CreateFromYawPitchRoll(-0.1f, 0.7f, -0.4f));
        transforms.SetLocalPose(frame, framePose);
        var observed = false;
        void OnInitialized(Entity<MetaDataComponent> entity)
        {
            var transform = entities.GetComponent<TransformComponent>(entity);
            Assert.That(transform.ParentUid, Is.EqualTo(frame));
            SamePose(transform.LocalPose, local);
            SamePose(transforms.GetWorldPose(entity), framePose.Compose(local));
            observed = true;
        }
        entities.EntityInitialized += OnInitialized;
        EntityUid child;
        try
        {
            child = entities.SpawnAttachedTo(null, frame, local);
        }
        finally
        {
            entities.EntityInitialized -= OnInitialized;
        }
        Assert.That(observed, Is.True);
        SamePose(transforms.GetWorldPose(child), framePose.Compose(local));
    }

    [Test]
    public void InvalidSpatialSpawnLeavesNoAllocatedEntityOrParentLink()
    {
        var sim = RobustServerSimulation.NewSimulation()
            .RegisterPrototypes(prototypes => prototypes.LoadString("""
                - type: entity
                  id: SpatialNoLocalRotation
                  components:
                  - type: Transform
                    noRot: true
                """))
            .InitializeInstance();
        var entities = sim.Resolve<IEntityManager>();
        var maps = entities.System<SharedMapSystem>();
        var map = maps.CreateMap();
        var removed = entities.SpawnAttachedTo(null, map, RigidTransform3.Identity);
        entities.DeleteEntity(removed);
        var count = entities.EntityCount;
        Assert.Throws<ArgumentOutOfRangeException>(() => entities.CreateEntityUninitialized(null, map, default(RigidTransform3)));
        Assert.Throws<ArgumentException>(() => entities.CreateEntityUninitialized(null, EntityUid.Invalid, RigidTransform3.Identity));
        Assert.Throws<ArgumentException>(() => entities.SpawnAttachedTo(null, removed, RigidTransform3.Identity));
        var rotated = new RigidTransform3(Vector3.UnitZ, Quaternion.CreateFromAxisAngle(Vector3.UnitX, 0.5f));
        Assert.Throws<InvalidOperationException>(() => entities.SpawnAttachedTo("SpatialNoLocalRotation", map, rotated));
        Assert.That(entities.EntityCount, Is.EqualTo(count));
        Assert.That(entities.GetComponent<TransformComponent>(map).ChildCount, Is.Zero);
        var valid = entities.SpawnAttachedTo("SpatialNoLocalRotation", map,
            new RigidTransform3(new Vector3(1, 2, 3), Quaternion.Identity));
        Assert.That(entities.GetComponent<TransformComponent>(valid).LocalPose.Position, Is.EqualTo(new Vector3(1, 2, 3)));
    }

    [Test]
    public void UninitializedSpatialSpawnsCanBeCancelledOrDeletedWithTheirParent()
    {
        var sim = RobustServerSimulation.NewSimulation().InitializeInstance();
        var entities = sim.Resolve<IEntityManager>();
        var map = entities.System<SharedMapSystem>().CreateMap();
        var parent = entities.SpawnAttachedTo(null, map, RigidTransform3.Identity);
        var pose = new RigidTransform3(new Vector3(1, 2, 3), Quaternion.Identity);
        var cancelled = entities.CreateEntityUninitialized(null, parent, pose);
        entities.DeleteEntity(cancelled);
        Assert.That(entities.EntityExists(cancelled), Is.False);
        Assert.That(entities.GetComponent<TransformComponent>(parent).ChildCount, Is.Zero);

        var child = entities.CreateEntityUninitialized(null, parent, pose);
        var grandchild = entities.CreateEntityUninitialized(null, child, pose);
        entities.DeleteEntity(parent);
        Assert.That(entities.EntityExists(parent), Is.False);
        Assert.That(entities.EntityExists(child), Is.False);
        Assert.That(entities.EntityExists(grandchild), Is.False);
        Assert.That(entities.GetComponent<TransformComponent>(map).ChildCount, Is.Zero);
    }
}
