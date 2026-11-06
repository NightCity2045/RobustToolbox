using System.Numerics;
using NUnit.Framework;
using Robust.Shared.Containers;
using Robust.Shared.GameObjects;
using Robust.Shared.Map.Components;
using Robust.Shared.Maths;
using Robust.Shared.Physics.Components;
using Robust.Shared.Spatial;
using Robust.Shared.Spatial.Physics;
using Robust.UnitTesting.Server;

namespace Robust.UnitTesting.Shared.GameObjects.Systems;

[TestFixture]
public sealed class SpatialPhysicsTests
{
    [Test]
    public void CanonicalGeometrySupportsAnActualEcsBodyAndRetainsHitIdentity()
    {
        var sim = RobustServerSimulation.NewSimulation().InitializeInstance();
        var entities = sim.Resolve<IEntityManager>();
        var physics = entities.System<SharedSpatialPhysicsSystem>();
        var world = entities.System<SharedSpatialSystem>().CreateWorld();
        physics.EnablePhysics(world);
        var frame = SpatialGeometryTests.CreateUninitializedFrame(entities, world);
        var slab = new SlabSurface(new SpatialSurfaceReference(frame, new SurfaceId(8)), new LayerId(5),
            new Box2(-10, -10, 10, 10), 0, 0.2f, entity: frame, stateRevision: 4);
        entities.System<SharedGeometrySystem>().ConfigureInitialGeometry(
            (frame, entities.GetComponent<SpatialGeometryComponent>(frame)), Array.Empty<BoundarySegment>(), new[] { slab }, 4);
        entities.InitializeAndStartEntity(frame, entities.GetComponent<MapComponent>(world).MapId);
        var body = SpawnBody(entities, world, new Vector3(0, 0, 5));
        Assert.That(physics.GetBodyCount(world), Is.EqualTo(2));
        for (var i = 0; i < 150; i++)
            physics.Update(1f / 30);
        var pose = entities.System<SharedTransformSystem>().GetWorldPose(body);
        Assert.That(pose.Position.Z, Is.EqualTo(2).Within(0.03));
        Assert.That(entities.GetComponent<SpatialBodyComponent>(body).LinearVelocity.Length(), Is.LessThan(0.05));
        var hit = physics.GetBackend(world).RayCast(new Vector3(3, 0, 3), -Vector3.UnitZ, 3, uint.MaxValue)!.Value;
        Assert.That(hit.Source.Entity, Is.EqualTo(frame));
        Assert.That(hit.Source.Surface, Is.EqualTo(slab.Surface));
        Assert.That(hit.Source.Revision, Is.EqualTo(4));
        entities.DeleteEntity(world);
        Assert.That(entities.EntityExists(body), Is.False);
        Assert.Throws<KeyNotFoundException>(() => physics.GetBackend(world));
    }

    [Test]
    public void FreeChildDoesNotReceiveParentMovementTwice()
    {
        var sim = RobustServerSimulation.NewSimulation().InitializeInstance();
        var entities = sim.Resolve<IEntityManager>();
        var physics = entities.System<SharedSpatialPhysicsSystem>();
        var transforms = entities.System<SharedTransformSystem>();
        var world = entities.System<SharedSpatialSystem>().CreateWorld();
        physics.EnablePhysics(world, new SpatialPhysicsSettings(Vector3.Zero, 1, 8));
        var parent = SpawnBody(entities, world, Vector3.Zero, new Vector3(2, 0, 0));
        var intermediate = entities.SpawnAttachedTo(null, parent, new RigidTransform3(Vector3.UnitZ,
            Quaternion.CreateFromAxisAngle(Vector3.UnitY, 0.4f)));
        var child = SpawnBody(entities, intermediate, new Vector3(0, 0, 3));
        var initial = transforms.GetWorldPose(child);
        var moveEvents = 0;
        void OnMove(ref MoveEvent ev)
        {
            if (ev.Sender != parent)
                return;
            Near(transforms.GetWorldPose(child).Position, initial.Position);
            moveEvents++;
        }
        transforms.OnGlobalMoveEvent += OnMove;
        for (var i = 0; i < 30; i++)
            physics.Update(1f / 30);
        transforms.OnGlobalMoveEvent -= OnMove;
        Assert.That(moveEvents, Is.GreaterThan(0));
        Assert.That(transforms.GetWorldPose(parent).Position.X, Is.EqualTo(2).Within(1e-5));
        Near(transforms.GetWorldPose(child).Position, initial.Position);
        Assert.That(entities.GetComponent<TransformComponent>(child).ParentUid, Is.EqualTo(intermediate));
        transforms.SetWorldPose(parent, new RigidTransform3(new Vector3(10, 20, 30),
            Quaternion.CreateFromAxisAngle(Vector3.UnitY, 0.5f)));
        Near(transforms.GetWorldPose(child).Position, initial.Position);
        physics.Update(1f / 30);
        Near(transforms.GetWorldPose(child).Position, initial.Position);
        entities.System<MetaDataSystem>().SetEntityPaused(child, true);
        physics.Update(1f / 30);
        Near(transforms.GetWorldPose(child).Position, initial.Position);
    }

    [Test]
    public void ContainerDormancyRestoresOneBodyAndWorldTransfersCleanUpBothSimulations()
    {
        var sim = RobustServerSimulation.NewSimulation().InitializeInstance();
        var entities = sim.Resolve<IEntityManager>();
        var physics = entities.System<SharedSpatialPhysicsSystem>();
        var transforms = entities.System<SharedTransformSystem>();
        var spatial = entities.System<SharedSpatialSystem>();
        var first = spatial.CreateWorld();
        var second = spatial.CreateWorld();
        physics.EnablePhysics(first, new SpatialPhysicsSettings(Vector3.Zero, 1, 8));
        physics.EnablePhysics(second, new SpatialPhysicsSettings(Vector3.Zero, 1, 8));
        var owner = entities.SpawnAttachedTo(null, first, new RigidTransform3(new Vector3(2, 3, 4), Quaternion.Identity));
        var body = SpawnBody(entities, first, new Vector3(0, 0, 5), Vector3.UnitX);
        var containers = entities.System<SharedContainerSystem>();
        var container = containers.EnsureContainer<Container>(owner, "spatial-physics");
        Assert.That(containers.Insert(body, container), Is.True);
        Assert.That(physics.GetBodyCount(first), Is.Zero);
        physics.Update(1f / 30);
        Assert.That(entities.GetComponent<TransformComponent>(body).LocalPose, Is.EqualTo(RigidTransform3.Identity));
        Assert.That(containers.Remove(body, container), Is.True);
        Assert.That(physics.GetBodyCount(first), Is.EqualTo(1));
        var pose = transforms.GetWorldPose(body);
        transforms.SetParent(body, second);
        Near(transforms.GetWorldPose(body).Position, pose.Position);
        Assert.That(physics.GetBodyCount(first), Is.Zero);
        Assert.That(physics.GetBodyCount(second), Is.EqualTo(1));
        physics.Update(1f / 30);
        Near(transforms.GetWorldPose(body).Position, pose.Position + Vector3.UnitX / 30);
        entities.DeleteEntity(first);
        Assert.That(entities.EntityExists(body), Is.True);
        entities.DeleteEntity(body);
        Assert.That(physics.GetBodyCount(second), Is.Zero);
    }

    [Test]
    public void ProfileAndOwnershipRejectionsLeaveTheExistingHierarchyAndBodyIntact()
    {
        var sim = RobustServerSimulation.NewSimulation().InitializeInstance();
        var entities = sim.Resolve<IEntityManager>();
        var physics = entities.System<SharedSpatialPhysicsSystem>();
        var spatial = entities.System<SharedSpatialSystem>();
        var world = spatial.CreateWorld();
        var legacy = spatial.CreateWorld();
        physics.EnablePhysics(world);
        var frame = entities.SpawnAttachedTo(null, world, RigidTransform3.Identity);
        var body = SpawnBody(entities, frame, new Vector3(0, 0, 5));
        var before = entities.System<SharedTransformSystem>().GetWorldPose(body);
        Assert.Throws<InvalidOperationException>(() => entities.System<SharedTransformSystem>().SetParent(frame, legacy));
        Assert.That(entities.GetComponent<TransformComponent>(frame).ParentUid, Is.EqualTo(world.Owner));
        Assert.That(entities.System<SharedTransformSystem>().GetWorldPose(body), Is.EqualTo(before));
        Assert.That(physics.GetBodyCount(world), Is.EqualTo(1));

        var planar = entities.SpawnAttachedTo(null, legacy, RigidTransform3.Identity);
        entities.AddComponent<PhysicsComponent>(planar);
        Assert.Throws<InvalidOperationException>(() => physics.EnablePhysics(legacy));
        Assert.That(entities.HasComponent<SpatialPhysicsWorldComponent>(legacy), Is.False);
        Assert.Throws<InvalidOperationException>(() => entities.System<SharedTransformSystem>().SetParent(planar, world));
        Assert.That(entities.GetComponent<TransformComponent>(planar).ParentUid, Is.EqualTo(legacy.Owner));
        var dual = entities.CreateEntityUninitialized(null, world, RigidTransform3.Identity);
        entities.AddComponent<PhysicsComponent>(dual);
        var component = entities.AddComponent<SpatialBodyComponent>(dual);
        Assert.Throws<InvalidOperationException>(() => physics.ConfigureInitialBody((dual, component), SphereDefinition()));
        entities.DeleteEntity(dual);
        Assert.That(physics.GetBodyCount(world), Is.EqualTo(1));
    }

    [Test]
    public void PausedBodyRemainsAColliderAndResumesItsCanonicalVelocity()
    {
        var sim = RobustServerSimulation.NewSimulation().InitializeInstance();
        var entities = sim.Resolve<IEntityManager>();
        var physics = entities.System<SharedSpatialPhysicsSystem>();
        var transforms = entities.System<SharedTransformSystem>();
        var world = entities.System<SharedSpatialSystem>().CreateWorld();
        physics.EnablePhysics(world, new SpatialPhysicsSettings(Vector3.Zero, 1, 8));
        var body = SpawnBody(entities, world, new Vector3(0, 0, 3), Vector3.UnitX);
        var metadata = entities.System<MetaDataSystem>();
        metadata.SetEntityPaused(body, true);
        physics.Update(1);
        Near(transforms.GetWorldPose(body).Position, new Vector3(0, 0, 3));
        Assert.That(physics.GetBackend(world).RayCast(new Vector3(0, 0, 5), -Vector3.UnitZ, 3, 1), Is.Not.Null);
        var component = entities.GetComponent<SpatialBodyComponent>(body);
        Assert.That(component.LinearVelocity, Is.EqualTo(Vector3.UnitX));
        physics.SetVelocity((body, component), Vector3.UnitY, Vector3.Zero);
        metadata.SetEntityPaused(body, false);
        physics.Update(1f / 30);
        Near(transforms.GetWorldPose(body).Position, new Vector3(0, 1f / 30, 3));
    }

    internal static SpatialBodyDefinition SphereDefinition()
        => new(SpatialMotionOwner.DynamicBody, new[]
        {
            new SpatialColliderDefinition(new SpatialSphereShape(0.25f), RigidTransform3.Identity)
        });

    [Test]
    public void PhysicsMoveObserversSeeTheSameTickVelocity()
    {
        var sim = RobustServerSimulation.NewSimulation().InitializeInstance();
        var entities = sim.Resolve<IEntityManager>();
        var world = entities.System<SharedSpatialSystem>().CreateWorld();
        var physics = entities.System<SharedSpatialPhysicsSystem>();
        physics.EnablePhysics(world);
        var body = SpawnBody(entities, world, new Vector3(0, 0, 3));
        var observed = false;
        void OnMove(ref MoveEvent ev)
        {
            if (ev.Sender != body)
                return;
            observed = true;
            Assert.That(entities.GetComponent<SpatialBodyComponent>(body).LinearVelocity.Z, Is.LessThan(0));
            Assert.That(entities.System<SharedTransformSystem>().GetWorldPose(body).Position.Z, Is.LessThan(3));
        }
        entities.System<SharedTransformSystem>().OnGlobalMoveEvent += OnMove;
        physics.Update(1f / 30);
        Assert.That(observed, Is.True);
    }

    [Test]
    public void InvalidPoseBatchDoesNotPartiallyCommitTransformsOrRelatedState()
    {
        var sim = RobustServerSimulation.NewSimulation().InitializeInstance();
        var entities = sim.Resolve<IEntityManager>();
        var world = entities.System<SharedSpatialSystem>().CreateWorld();
        var parent = entities.SpawnAttachedTo(null, world, RigidTransform3.Identity);
        var child = entities.SpawnAttachedTo(null, parent, new RigidTransform3(Vector3.UnitZ, Quaternion.Identity));
        var transforms = entities.System<SharedTransformSystem>();
        var parentBefore = transforms.GetWorldPose(parent);
        var childBefore = transforms.GetWorldPose(child);
        var relatedCommitted = false;
        Assert.Throws<ArgumentOutOfRangeException>(() => transforms.CommitSpatialPoses(
            new Dictionary<EntityUid, RigidTransform3>
            {
                [parent] = new(new Vector3(10, 0, 0), Quaternion.Identity), [child] = default
            }, () => relatedCommitted = true));
        Assert.That(relatedCommitted, Is.False);
        Assert.That(transforms.GetWorldPose(parent), Is.EqualTo(parentBefore));
        Assert.That(transforms.GetWorldPose(child), Is.EqualTo(childBefore));
    }

    internal static EntityUid SpawnBody(IEntityManager entities, EntityUid parent, Vector3 position, Vector3 velocity = default)
    {
        var body = entities.CreateEntityUninitialized(null, parent, new RigidTransform3(position, Quaternion.Identity));
        var component = entities.AddComponent<SpatialBodyComponent>(body);
        entities.System<SharedSpatialPhysicsSystem>().ConfigureInitialBody((body, component), SphereDefinition(), velocity);
        entities.InitializeAndStartEntity(body, entities.GetComponent<TransformComponent>(parent).MapID);
        return body;
    }

    private static void Near(Vector3 actual, Vector3 expected)
        => Assert.That(Vector3.Distance(actual, expected), Is.LessThan(1e-4));
}
