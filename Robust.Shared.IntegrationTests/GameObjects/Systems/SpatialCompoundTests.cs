using System.Numerics;
using Moq;
using NUnit.Framework;
using Robust.Shared.Containers;
using Robust.Shared.GameObjects;
using Robust.Shared.Maths;
using Robust.Shared.Spatial;
using Robust.Shared.Spatial.Physics;
using Robust.Shared.Timing;
using Robust.UnitTesting.Server;

namespace Robust.UnitTesting.Shared.GameObjects.Systems;

[TestFixture]
public sealed class SpatialCompoundTests
{
    [Test]
    public void AnchoredMassImpulseMovementTransferAndDeletionUseOneBody()
    {
        var sim = RobustServerSimulation.NewSimulation().InitializeInstance();
        var entities = sim.Resolve<IEntityManager>();
        var spatial = entities.System<SharedSpatialSystem>();
        var physics = entities.System<SharedSpatialPhysicsSystem>();
        var transforms = entities.System<SharedTransformSystem>();
        var a = spatial.CreateWorld();
        var b = spatial.CreateWorld();
        physics.EnablePhysics(a, new SpatialPhysicsSettings(Vector3.Zero, 1, 8));
        physics.EnablePhysics(b, new SpatialPhysicsSettings(Vector3.Zero, 1, 8));
        var root = Body(entities, a, Vector3.UnitZ * 4, SpatialMotionOwner.DynamicBody, 2);
        var child = Body(entities, root, Vector3.UnitX * 4, SpatialMotionOwner.AnchoredCompoundChild, 6);
        Assert.That(physics.GetBodyCount(a), Is.EqualTo(1));
        var childBody = entities.GetComponent<SpatialBodyComponent>(child);
        physics.ApplyImpulse((child, childBody), Vector3.UnitY * 8, new Vector3(3, 0, 4));
        var rootBody = entities.GetComponent<SpatialBodyComponent>(root);
        Assert.That(Vector3.Distance(rootBody.LinearVelocity, Vector3.UnitY), Is.LessThan(1e-5));
        Assert.That(rootBody.AngularVelocity.Length(), Is.LessThan(1e-5));
        Assert.That(childBody.LinearVelocity, Is.EqualTo(Vector3.Zero), "The anchored child has no independent velocity authority.");
        Assert.That(physics.GetVelocityAtWorldPoint((child, childBody), transforms.GetWorldPose(child).Position), Is.EqualTo(Vector3.UnitY));

        var passenger = SpatialPhysicsTests.SpawnBody(entities, root, Vector3.UnitZ * 5);
        var passengerPose = transforms.GetWorldPose(passenger);
        transforms.SetWorldPose(root, new RigidTransform3(new Vector3(20, 0, 4), Quaternion.CreateFromAxisAngle(Vector3.UnitZ, MathF.PI / 2)));
        Assert.That(Vector3.Distance(transforms.GetWorldPose(child).Position, new Vector3(20, 4, 4)), Is.LessThan(1e-4));
        Assert.That(Vector3.Distance(transforms.GetWorldPose(passenger).Position, passengerPose.Position), Is.LessThan(1e-4),
            "Free passengers must not inherit the root displacement twice.");
        Assert.That(MathF.Abs(Quaternion.Dot(transforms.GetWorldPose(passenger).Rotation, passengerPose.Rotation)), Is.EqualTo(1).Within(1e-5));
        var queries = entities.System<SpatialQuerySystem>();
        Assert.That(queries.RayCast(new SpatialQueryContext(a, a, "test", uint.MaxValue), new Vector3(20, 4, 6), -Vector3.UnitZ, 5)!.Value.Entity,
            Is.EqualTo(child));
        Assert.Throws<InvalidOperationException>(() => transforms.SetParent(child, a));
        Assert.That(entities.GetComponent<TransformComponent>(child).ParentUid, Is.EqualTo(root));
        transforms.SetParent(root, b);
        Assert.That(physics.GetBodyCount(a), Is.Zero);
        Assert.That(physics.GetBodyCount(b), Is.EqualTo(2));
        Assert.That(Vector3.Distance(transforms.GetWorldPose(passenger).Position, passengerPose.Position), Is.LessThan(1e-4));
        entities.DeleteEntity(child);
        Assert.That(physics.GetBodyCount(b), Is.EqualTo(2));
        physics.ApplyImpulse((root, rootBody), Vector3.UnitY * 2, transforms.GetWorldPose(root).Position);
        Assert.That(Vector3.Distance(rootBody.LinearVelocity, Vector3.UnitY * 2), Is.LessThan(1e-4), "Removed child mass cannot remain in the hull.");
        entities.DeleteEntity(root);
        Assert.That(physics.GetBodyCount(b), Is.Zero);
    }

    [Test]
    public void MultipleAnchoredGeometryEditsPrepareOneCompleteHullAndRejectMembershipDrift()
    {
        var sim = RobustServerSimulation.NewSimulation()
            .RegisterEntitySystems(f => f.LoadExtraSystemType<SpatialHistorySystem>()).InitializeInstance();
        var entities = sim.Resolve<IEntityManager>();
        var physics = entities.System<SharedSpatialPhysicsSystem>();
        var geometry = entities.System<SharedGeometrySystem>();
        var world = entities.System<SharedSpatialSystem>().CreateWorld();
        physics.EnablePhysics(world, new SpatialPhysicsSettings(Vector3.Zero, 1, 8));
        var root = Body(entities, world, Vector3.UnitZ * 4, SpatialMotionOwner.DynamicBody, 2);
        var left = Geometry(entities, root, -3);
        var right = Geometry(entities, root, 3);
        var history = entities.System<SpatialHistorySystem>();
        history.Configure(world, new SpatialHistorySettings(10, 1000, 1000));
        Mock.Get(sim.Resolve<IGameTiming>()).SetupGet(t => t.CurTick).Returns(new GameTick(1));
        Assert.That(physics.GetBodyCount(world), Is.EqualTo(1));
        using var edit = geometry.PrepareTransaction(new[] { Raise(left, 1), Raise(right, 2) });
        geometry.QueueTransaction(edit);
        physics.Update(1f / 60);
        Assert.That(edit.Status, Is.EqualTo(WorldEditStatus.Committed));
        Assert.That(physics.GetBodyCount(world), Is.EqualTo(1));
        Assert.That(history.TryGetScene(world, new GameTick(1), out var historical), Is.True);
        Assert.That(historical!.TryGetGeometry(left, out var oldLeft), Is.True,
            "The root can use explicit colliders while its anchored child supplies historical geometry.");
        Assert.That(oldLeft!.Revision, Is.EqualTo(1));
        Assert.That(historical.TryGetBodyState(left, out var leftState), Is.True);
        Assert.That(Vector3.Distance(leftState.Pose.Position, new Vector3(-3, 0, 4)), Is.LessThan(1e-4));
        var queries = entities.System<SpatialQuerySystem>();
        var context = new SpatialQueryContext(world, world, "test", uint.MaxValue);
        Assert.That(queries.FindSurface(context, new Vector3(-3, 0, 10), -Vector3.UnitZ, 10)!.Value.Point!.Value.Z,
            Is.EqualTo(5).Within(1e-4));
        Assert.That(queries.FindSurface(context, new Vector3(3, 0, 10), -Vector3.UnitZ, 10)!.Value.Point!.Value.Z,
            Is.EqualTo(6).Within(1e-4));
        using var stale = geometry.PrepareTransaction(new[] { Raise(left, 3), Raise(right, 4) });
        entities.System<SharedTransformSystem>().SetLocalPose(left, new RigidTransform3(new Vector3(-6, 0, 0), Quaternion.Identity));
        geometry.QueueTransaction(stale);
        physics.Update(1f / 60);
        Assert.That(stale.Status, Is.EqualTo(WorldEditStatus.Rejected));
        Assert.That(geometry.CaptureSnapshot(left).Revision, Is.EqualTo(1));
        Assert.That(geometry.CaptureSnapshot(right).Revision, Is.EqualTo(1));
        Assert.That(physics.GetBodyCount(world), Is.EqualTo(1));

        SpatialGeometryEdit Raise(EntityUid frame, float height) => new(geometry.CaptureSnapshot(frame),
            layers: new Dictionary<LayerId, LayerDefinition> { [new LayerId(1)] = new(height, "Floor") });
    }

    [Test]
    public void ReconfigurationAttachesAndDetachesWithoutDuplicatingMotionOrBodies()
    {
        var sim = RobustServerSimulation.NewSimulation().InitializeInstance();
        var entities = sim.Resolve<IEntityManager>();
        var physics = entities.System<SharedSpatialPhysicsSystem>();
        var transforms = entities.System<SharedTransformSystem>();
        var world = entities.System<SharedSpatialSystem>().CreateWorld();
        physics.EnablePhysics(world, new SpatialPhysicsSettings(Vector3.Zero, 1, 8));
        var root = Body(entities, world, Vector3.Zero, SpatialMotionOwner.DynamicBody, 10);
        var child = Body(entities, root, Vector3.UnitX * 3, SpatialMotionOwner.DynamicBody, 2);
        var rootBody = entities.GetComponent<SpatialBodyComponent>(root);
        var component = entities.GetComponent<SpatialBodyComponent>(child);
        physics.SetVelocity((root, rootBody), Vector3.UnitX, Vector3.UnitZ);
        var anchored = new SpatialBodyDefinition(SpatialMotionOwner.AnchoredCompoundChild, component.Definition.Colliders, mass: 2);
        physics.ReconfigureBody((child, component), anchored, 0, Vector3.Zero, Vector3.Zero);
        Assert.That(physics.GetBodyCount(world), Is.EqualTo(1));
        Assert.That(Vector3.Distance(physics.GetVelocityAtWorldPoint((child, component), transforms.GetWorldPose(child).Position),
            new Vector3(1, 3, 0)), Is.LessThan(1e-5));
        var before = transforms.GetWorldPose(child);
        var velocity = physics.GetVelocityAtWorldPoint((child, component), before.Position);
        var dynamic = new SpatialBodyDefinition(SpatialMotionOwner.DynamicBody, component.Definition.Colliders, mass: 2);
        physics.ReconfigureBody((child, component), dynamic, 1, velocity, rootBody.AngularVelocity);
        Assert.That(physics.GetBodyCount(world), Is.EqualTo(2));
        physics.Update(1f / 60);
        Assert.That(Vector3.Distance(transforms.GetWorldPose(child).Position, before.Position + velocity / 60), Is.LessThan(1e-4));
        Assert.That(component.LinearVelocity, Is.EqualTo(velocity));
        physics.ReconfigureBody((child, component), anchored, 2, Vector3.Zero, Vector3.Zero);
        Assert.That(physics.GetBodyCount(world), Is.EqualTo(1));
        Assert.That(rootBody.LinearVelocity, Is.EqualTo(Vector3.UnitX));
    }

    [Test]
    public void ContainerDormancyAndChangingRootsRemoveOldCompoundMass()
    {
        var sim = RobustServerSimulation.NewSimulation().InitializeInstance();
        var entities = sim.Resolve<IEntityManager>();
        var physics = entities.System<SharedSpatialPhysicsSystem>();
        var transforms = entities.System<SharedTransformSystem>();
        var world = entities.System<SharedSpatialSystem>().CreateWorld();
        physics.EnablePhysics(world, new SpatialPhysicsSettings(Vector3.Zero, 1, 8));
        var a = Body(entities, world, Vector3.Zero, SpatialMotionOwner.DynamicBody, 2);
        var b = Body(entities, world, Vector3.UnitX * 10, SpatialMotionOwner.DynamicBody, 3);
        var child = Body(entities, a, Vector3.UnitX * 3, SpatialMotionOwner.AnchoredCompoundChild, 4);
        var box = entities.SpawnAttachedTo(null, a, new RigidTransform3(Vector3.UnitX * 2, Quaternion.Identity));
        var containers = entities.System<SharedContainerSystem>();
        var container = containers.EnsureContainer<Container>(box, "compound-test");
        Assert.That(containers.Insert(child, container), Is.True);
        var childBody = entities.GetComponent<SpatialBodyComponent>(child);
        Assert.Throws<InvalidOperationException>(() => physics.ApplyImpulse((child, childBody), Vector3.UnitY, Vector3.Zero));
        var aBody = entities.GetComponent<SpatialBodyComponent>(a);
        physics.ApplyImpulse((a, aBody), Vector3.UnitY * 2, Vector3.Zero);
        Assert.That(Vector3.Distance(aBody.LinearVelocity, Vector3.UnitY), Is.LessThan(1e-4));
        Assert.That(containers.Remove(child, container), Is.True);
        var centerA = transforms.GetWorldPose(child).Position * (4f / 6);
        physics.ApplyImpulse((a, aBody), Vector3.UnitY * 6, centerA);
        Assert.That(Vector3.Distance(aBody.LinearVelocity, Vector3.UnitY * 2), Is.LessThan(1e-4));
        var before = transforms.GetWorldPose(child);
        transforms.SetParent(child, b);
        Assert.That(Vector3.Distance(transforms.GetWorldPose(child).Position, before.Position), Is.LessThan(1e-4));
        var bBody = entities.GetComponent<SpatialBodyComponent>(b);
        var centerB = (transforms.GetWorldPose(b).Position * 3 + before.Position * 4) / 7;
        physics.ApplyImpulse((b, bBody), Vector3.UnitY * 7, centerB);
        Assert.That(Vector3.Distance(bBody.LinearVelocity, Vector3.UnitY), Is.LessThan(1e-4));
        physics.ApplyImpulse((a, aBody), Vector3.UnitY * 2, Vector3.Zero);
        Assert.That(Vector3.Distance(aBody.LinearVelocity, Vector3.UnitY * 3), Is.LessThan(1e-4));
        entities.DeleteEntity(a);
        Assert.That(entities.EntityExists(child), Is.True);
        Assert.That(physics.GetBodyCount(world), Is.EqualTo(1));
    }

    [Test]
    public void SeatConstraintResolvesAnchoredOffsetsAndSurvivesHullRebuild()
    {
        var sim = RobustServerSimulation.NewSimulation().InitializeInstance();
        var entities = sim.Resolve<IEntityManager>();
        var physics = entities.System<SharedSpatialPhysicsSystem>();
        var transforms = entities.System<SharedTransformSystem>();
        var world = entities.System<SharedSpatialSystem>().CreateWorld();
        physics.EnablePhysics(world, new SpatialPhysicsSettings(Vector3.Zero, 1, 8));
        var root = Body(entities, world, Vector3.Zero, SpatialMotionOwner.DynamicBody, 100);
        var seat = Body(entities, root, Vector3.UnitX * 3, SpatialMotionOwner.AnchoredCompoundChild, 2);
        var passenger = SpatialPhysicsTests.SpawnBody(entities, world, new Vector3(3, 0, 2));
        var link = entities.CreateEntityUninitialized(null, world, RigidTransform3.Identity);
        var constraint = entities.AddComponent<SpatialConstraintComponent>(link);
        physics.ConfigureInitialConstraint((link, constraint), new SpatialConstraintDefinition(seat, passenger,
            SpatialConstraintKind.Fixed, relativePose: new RigidTransform3(Vector3.UnitZ * 2, Quaternion.Identity)));
        entities.InitializeAndStartEntity(link);
        var rootBody = entities.GetComponent<SpatialBodyComponent>(root);
        physics.SetVelocity((root, rootBody), Vector3.UnitX, Vector3.UnitZ * 0.3f);
        for (var i = 0; i < 60; i++) physics.Update(1f / 60);
        CheckSeat();
        var seatBody = entities.GetComponent<SpatialBodyComponent>(seat);
        physics.ReconfigureBody((seat, seatBody), new SpatialBodyDefinition(SpatialMotionOwner.AnchoredCompoundChild,
            seatBody.Definition.Colliders, mass: 4), 0, Vector3.Zero, Vector3.Zero);
        for (var i = 0; i < 60; i++) physics.Update(1f / 60);
        CheckSeat();
        entities.DeleteEntity(seat);
        Assert.That(entities.HasComponent<SpatialConstraintComponent>(link), Is.False);
        Assert.That(entities.EntityExists(passenger), Is.True);
        Assert.That(physics.GetBodyCount(world), Is.EqualTo(2));

        void CheckSeat()
        {
            var expected = transforms.GetWorldPose(seat).Compose(new RigidTransform3(Vector3.UnitZ * 2, Quaternion.Identity));
            var actual = transforms.GetWorldPose(passenger);
            Assert.That(Vector3.Distance(actual.Position, expected.Position), Is.LessThan(0.03));
            Assert.That(MathF.Abs(Quaternion.Dot(actual.Rotation, expected.Rotation)), Is.GreaterThan(0.999));
        }
    }

    private static EntityUid Body(IEntityManager entities, EntityUid parent, Vector3 position, SpatialMotionOwner owner, float mass)
    {
        var uid = entities.CreateEntityUninitialized(null, parent, new RigidTransform3(position, Quaternion.Identity));
        var body = entities.AddComponent<SpatialBodyComponent>(uid);
        entities.System<SharedSpatialPhysicsSystem>().ConfigureInitialBody((uid, body),
            new SpatialBodyDefinition(owner, new[] { new SpatialColliderDefinition(new SpatialSphereShape(0.25f), RigidTransform3.Identity) }, mass: mass));
        entities.InitializeAndStartEntity(uid);
        return uid;
    }

    private static EntityUid Geometry(IEntityManager entities, EntityUid parent, float x)
    {
        var uid = entities.CreateEntityUninitialized(null, parent, new RigidTransform3(Vector3.UnitX * x, Quaternion.Identity));
        var frame = entities.AddComponent<SpatialFrameComponent>(uid);
        var layer = new LayerId(1);
        entities.System<SharedSpatialSystem>().ConfigureInitialLayers((uid, frame),
            new Dictionary<LayerId, LayerDefinition> { [layer] = new(0, "Floor") });
        var geometry = entities.AddComponent<SpatialGeometryComponent>(uid);
        entities.System<SharedGeometrySystem>().ConfigureInitialGeometry((uid, geometry), Array.Empty<BoundarySegment>(),
            new[] { new SlabSurface(new SpatialSurfaceReference(uid, new SurfaceId(1)), layer, new Box2(-0.4f, -0.4f, 0.4f, 0.4f), 0, 0.2f) });
        var body = entities.AddComponent<SpatialBodyComponent>(uid);
        entities.System<SharedSpatialPhysicsSystem>().ConfigureInitialBody((uid, body),
            new SpatialBodyDefinition(SpatialMotionOwner.AnchoredCompoundChild, useGeometry: true, mass: 3));
        entities.InitializeAndStartEntity(uid);
        return uid;
    }
}
