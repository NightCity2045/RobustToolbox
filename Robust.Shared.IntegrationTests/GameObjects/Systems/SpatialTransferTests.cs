using System.Numerics;
using NUnit.Framework;
using Robust.Shared.Containers;
using Robust.Shared.GameObjects;
using Robust.Shared.Maths;
using Robust.Shared.Reflection;
using Robust.Shared.Spatial;
using Robust.Shared.Spatial.Physics;
using Robust.UnitTesting.Server;

namespace Robust.UnitTesting.Shared.GameObjects.Systems;

[TestFixture]
public sealed class SpatialTransferTests
{
    [TestCase(false)]
    [TestCase(true)]
    public void ClosedGraphMovesOnceAndObserversSeeCompleteDestination(bool paused)
    {
        var sim = RobustServerSimulation.NewSimulation()
            .RegisterEntitySystems(f => f.LoadExtraSystemType<TransferObserver>()).InitializeInstance();
        var entities = sim.Resolve<IEntityManager>();
        var physics = entities.System<SharedSpatialPhysicsSystem>();
        var transforms = entities.System<SharedTransformSystem>();
        var source = entities.System<SharedSpatialSystem>().CreateWorld();
        var destination = entities.System<SharedSpatialSystem>().CreateWorld();
        physics.EnablePhysics(source, new SpatialPhysicsSettings(Vector3.Zero, 1, 8));
        physics.EnablePhysics(destination, new SpatialPhysicsSettings(Vector3.Zero, 1, 8));
        entities.System<SharedMapSystem>().SetPaused(destination.Owner, paused);
        var root = SpatialPhysicsTests.SpawnBody(entities, source, Vector3.UnitZ * 4, Vector3.UnitX);
        var part = entities.CreateEntityUninitialized(null, root, new RigidTransform3(Vector3.UnitX * 3, Quaternion.Identity));
        var partBody = entities.AddComponent<SpatialBodyComponent>(part);
        physics.ConfigureInitialBody((part, partBody), new SpatialBodyDefinition(SpatialMotionOwner.AnchoredCompoundChild,
            new[] { new SpatialColliderDefinition(new SpatialSphereShape(0.25f), RigidTransform3.Identity) }, mass: 2));
        entities.InitializeAndStartEntity(part);
        var passenger = SpatialPhysicsTests.SpawnBody(entities, source, new Vector3(3, 0, 6), Vector3.UnitX);
        var link = SpatialConstraintTests.SpawnConstraint(entities, source, new SpatialConstraintDefinition(part, passenger,
            SpatialConstraintKind.Fixed, relativePose: new RigidTransform3(Vector3.UnitZ * 2, Quaternion.Identity)));
        var box = entities.SpawnAttachedTo(null, root, new RigidTransform3(Vector3.UnitY, Quaternion.Identity));
        var containers = entities.System<SharedContainerSystem>();
        var container = containers.EnsureContainer<Container>(box, "hold");
        var cargo = SpatialPhysicsTests.SpawnBody(entities, source, Vector3.UnitZ * 8, Vector3.UnitY * 2);
        Assert.That(containers.Insert(cargo, container), Is.True);
        var members = new[] { root, part, passenger, link, box, cargo };
        var original = members.ToDictionary(uid => uid, transforms.GetWorldPose);
        var mapping = new RigidTransform3(new Vector3(100, 200, 50), Quaternion.CreateFromAxisAngle(Vector3.UnitZ, MathF.PI / 2));
        var observer = entities.System<TransferObserver>();
        observer.Check = () =>
        {
            foreach (var uid in members)
            {
                Assert.That(entities.GetComponent<TransformComponent>(uid).MapUid, Is.EqualTo(destination.Owner));
                Assert.That(entities.GetComponent<MetaDataComponent>(uid).EntityPaused, Is.EqualTo(paused));
                Assert.That(Vector3.Distance(transforms.GetWorldPose(uid).Position, mapping.Compose(original[uid]).Position), Is.LessThan(1e-4));
            }
            Assert.That(physics.GetBodyCount(source), Is.Zero);
            Assert.That(physics.GetBodyCount(destination), Is.EqualTo(2));
            Assert.That(physics.IsConstraintActive(link), Is.EqualTo(!paused));
            Assert.Throws<InvalidOperationException>(() => physics.Update(1f / 60));
            Assert.Throws<InvalidOperationException>(() => physics.TransferGraph(new[] { root }, source, RigidTransform3.Identity));
        };
        foreach (var uid in members) entities.System<MetaDataSystem>().AddFlag(uid, MetaDataFlags.ExtraTransformEvents);
        physics.TransferGraph(new[] { root, passenger, link }, destination, mapping);
        Assert.That(observer.Notifications, Is.GreaterThanOrEqualTo(members.Length + 1));
        Assert.That(container.Contains(cargo), Is.True);
        Assert.That(entities.GetComponent<TransformComponent>(part).ParentUid, Is.EqualTo(root));
        Assert.That(entities.GetComponent<TransformComponent>(cargo).ParentUid, Is.EqualTo(box));
        Assert.That(Vector3.Distance(entities.GetComponent<SpatialBodyComponent>(root).LinearVelocity, Vector3.UnitY), Is.LessThan(1e-5));
        Assert.That(Vector3.Distance(entities.GetComponent<SpatialBodyComponent>(cargo).LinearVelocity, -Vector3.UnitX * 2), Is.LessThan(1e-5));
        observer.Check = null;
        physics.TransferGraph(new[] { root, passenger, link }, source, mapping.Inverse());
        Assert.That(physics.GetBodyCount(destination), Is.Zero);
        Assert.That(physics.GetBodyCount(source), Is.EqualTo(2));
        Assert.That(physics.IsConstraintActive(link), Is.True);
        foreach (var uid in members)
            Assert.That(Vector3.Distance(transforms.GetWorldPose(uid).Position, original[uid].Position), Is.LessThan(1e-4));
        Assert.That(containers.Remove(cargo, container), Is.True);
        Assert.That(physics.GetBodyCount(source), Is.EqualTo(3));
        entities.DeleteEntity(destination);
        Assert.That(entities.EntityExists(passenger), Is.True);
        Assert.DoesNotThrow(() => physics.Update(1f / 60));
    }

    [Test]
    public void IncompleteConnectionOrOverlappingSelectionRejectsBeforeMutation()
    {
        var sim = RobustServerSimulation.NewSimulation().InitializeInstance();
        var entities = sim.Resolve<IEntityManager>();
        var physics = entities.System<SharedSpatialPhysicsSystem>();
        var spatial = entities.System<SharedSpatialSystem>();
        var source = spatial.CreateWorld();
        var destination = spatial.CreateWorld();
        physics.EnablePhysics(source, new SpatialPhysicsSettings(Vector3.Zero, 1, 8));
        physics.EnablePhysics(destination, new SpatialPhysicsSettings(Vector3.Zero, 1, 8));
        var a = SpatialPhysicsTests.SpawnBody(entities, source, Vector3.Zero);
        var b = SpatialPhysicsTests.SpawnBody(entities, a, Vector3.UnitX * 3);
        var link = SpatialConstraintTests.SpawnConstraint(entities, source,
            new SpatialConstraintDefinition(a, b, SpatialConstraintKind.Point));
        physics.Update(1f / 60);
        var pose = entities.System<SharedTransformSystem>().GetWorldPose(a);
        Assert.Throws<ArgumentException>(() => physics.TransferGraph(new[] { a, a, link }, destination, RigidTransform3.Identity));
        foreach (var roots in new[] { new[] { a }, new[] { link }, new[] { a, b, link } })
        {
            Assert.Throws<InvalidOperationException>(() => physics.TransferGraph(roots, destination, RigidTransform3.Identity),
                "A failed transfer may not partially move the graph.");
            Assert.That(entities.GetComponent<TransformComponent>(a).MapUid, Is.EqualTo(source.Owner));
            Assert.That(entities.System<SharedTransformSystem>().GetWorldPose(a), Is.EqualTo(pose));
            Assert.That(physics.IsConstraintActive(link), Is.True);
            Assert.That(physics.GetBodyCount(source), Is.EqualTo(2));
            Assert.That(physics.GetBodyCount(destination), Is.Zero);
        }
        physics.TransferGraph(new[] { a, link }, destination, RigidTransform3.Identity);
        Assert.That(physics.IsConstraintActive(link), Is.True);
        entities.DeleteEntity(source);
        Assert.That(entities.EntityExists(b), Is.True);
    }

    [Test]
    public void MovingSupportAndMotorKeepOneWitnessWhileLeftBehindMotorLosesSupport()
    {
        var sim = RobustServerSimulation.NewSimulation().InitializeInstance();
        var entities = sim.Resolve<IEntityManager>();
        var physics = entities.System<SharedSpatialPhysicsSystem>();
        var source = entities.System<SharedSpatialSystem>().CreateWorld();
        var destination = entities.System<SharedSpatialSystem>().CreateWorld();
        physics.EnablePhysics(source);
        physics.EnablePhysics(destination);
        var floor = SpatialMotorTests.Floor(entities, source);
        var rider = SpatialMotorTests.SpawnMotor(entities, source, new Vector3(0, 0, 0.8f));
        var remaining = SpatialMotorTests.SpawnMotor(entities, source, new Vector3(4, 0, 0.8f));
        for (var i = 0; i < 10; i++) physics.Update(1f / 60);
        var support = rider.Comp.Support!.Value;
        Assert.That(remaining.Comp.Support, Is.Not.Null);
        physics.SetMotorIntent(rider, new SpatialMotorIntent(Vector3.UnitX, SpatialMotorMode.Walking));
        var mapping = new RigidTransform3(Vector3.UnitZ * 10, Quaternion.CreateFromAxisAngle(Vector3.UnitZ, MathF.PI / 2));
        physics.TransferGraph(new[] { floor, rider.Owner }, destination, mapping);
        Assert.That(remaining.Comp.Support, Is.Null);
        Assert.That(rider.Comp.Support!.Value.Source, Is.EqualTo(support.Source));
        Assert.That(Vector3.Distance(rider.Comp.Support.Value.Point, mapping.TransformPoint(support.Point)), Is.LessThan(1e-4));
        Assert.That(Vector3.Distance(rider.Comp.Intent.Velocity, Vector3.UnitY), Is.LessThan(1e-5));
        physics.Update(1f / 60);
        Assert.That(rider.Comp.Support, Is.Not.Null);
        Assert.That(remaining.Comp.Support, Is.Null);
    }

    [Test]
    public void GeometryReferencesMustMoveTogetherInBothDirections()
    {
        var sim = RobustServerSimulation.NewSimulation().InitializeInstance();
        var entities = sim.Resolve<IEntityManager>();
        var physics = entities.System<SharedSpatialPhysicsSystem>();
        var source = entities.System<SharedSpatialSystem>().CreateWorld();
        var destination = entities.System<SharedSpatialSystem>().CreateWorld();
        physics.EnablePhysics(source);
        physics.EnablePhysics(destination);
        var part = entities.SpawnAttachedTo(null, source, RigidTransform3.Identity);
        var frame = SpatialGeometryTests.CreateUninitializedFrame(entities, source);
        var surface = new SpatialSurfaceReference(frame, new SurfaceId(9));
        entities.System<SharedGeometrySystem>().ConfigureInitialGeometry(
            (frame, entities.GetComponent<SpatialGeometryComponent>(frame)), Array.Empty<BoundarySegment>(),
            new[] { new SlabSurface(surface, new LayerId(5), new Box2(-1, -1, 1, 1), 0, 0.2f, entity: part) });
        entities.InitializeAndStartEntity(frame);
        foreach (var root in new[] { frame, part })
        {
            Assert.Throws<InvalidOperationException>(() => physics.TransferGraph(new[] { root }, destination, RigidTransform3.Identity));
            Assert.That(entities.GetComponent<TransformComponent>(root).MapUid, Is.EqualTo(source.Owner));
            Assert.That(physics.GetBodyCount(destination), Is.Zero);
        }
        physics.TransferGraph(new[] { frame, part }, destination, RigidTransform3.Identity);
        var hit = entities.System<SpatialQuerySystem>().RayCast(new SpatialQueryContext(destination, destination, "test", uint.MaxValue),
            new Vector3(0, 0, 5), -Vector3.UnitZ, 10);
        Assert.That(hit!.Value.Entity, Is.EqualTo(part));
        Assert.That(hit.Value.Surface, Is.EqualTo(surface));
        entities.DeleteEntity(source);
        Assert.That(entities.EntityExists(part), Is.True);
    }

    [Test]
    public void UnrepresentableDescendantPoseRejectsWithoutChangingWorldOrBodies()
    {
        var sim = RobustServerSimulation.NewSimulation().InitializeInstance();
        var entities = sim.Resolve<IEntityManager>();
        var physics = entities.System<SharedSpatialPhysicsSystem>();
        var source = entities.System<SharedSpatialSystem>().CreateWorld();
        var destination = entities.System<SharedSpatialSystem>().CreateWorld();
        physics.EnablePhysics(source);
        physics.EnablePhysics(destination);
        var root = SpatialPhysicsTests.SpawnBody(entities, source, Vector3.Zero);
        var child = entities.SpawnAttachedTo(null, root, new RigidTransform3(Vector3.UnitX * float.MaxValue, Quaternion.Identity));
        Assert.Throws<ArithmeticException>(() => physics.TransferGraph(new[] { root }, destination,
            new RigidTransform3(Vector3.UnitX * float.MaxValue, Quaternion.Identity)));
        Assert.That(entities.GetComponent<TransformComponent>(root).MapUid, Is.EqualTo(source.Owner));
        Assert.That(entities.GetComponent<TransformComponent>(child).ParentUid, Is.EqualTo(root));
        Assert.That(physics.GetBodyCount(source), Is.EqualTo(1));
        Assert.That(physics.GetBodyCount(destination), Is.Zero);
    }

    [Reflect(false)]
    private sealed class TransferObserver : EntitySystem
    {
        public Action? Check;
        public int Notifications;
        public override void Initialize()
        {
            SubscribeLocalEvent<TransformComponent, MapUidChangedEvent>(OnMap);
            SubscribeLocalEvent<TransformComponent, EntityPausedEvent>(OnPause);
            SubscribeLocalEvent<TransformComponent, EntityUnpausedEvent>(OnUnpause);
            SubscribeLocalEvent<SpatialGraphTransferred>(OnCompleted);
        }
        private void Observe() { if (Check == null) return; Notifications++; Check(); }
        private void OnMap(EntityUid uid, TransformComponent comp, ref MapUidChangedEvent ev) => Observe();
        private void OnPause(EntityUid uid, TransformComponent comp, ref EntityPausedEvent ev) => Observe();
        private void OnUnpause(EntityUid uid, TransformComponent comp, ref EntityUnpausedEvent ev) => Observe();
        private void OnCompleted(ref SpatialGraphTransferred ev) => Observe();
    }
}
