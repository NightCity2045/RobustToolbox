using System.Numerics;
using NUnit.Framework;
using Robust.Shared.Containers;
using Robust.Shared.GameObjects;
using Robust.Shared.Map.Components;
using Robust.Shared.Maths;
using Robust.Shared.Reflection;
using Robust.Shared.Spatial;
using Robust.Shared.Spatial.Physics;
using Robust.UnitTesting.Server;

namespace Robust.UnitTesting.Shared.GameObjects.Systems;

[TestFixture]
public sealed class SpatialMotorTests
{
    internal static Entity<SpatialMotorComponent> SpawnMotor(IEntityManager entities, EntityUid parent, Vector3 position,
        SpatialMotorIntent intent = default)
    {
        var physics = entities.System<SharedSpatialPhysicsSystem>();
        var uid = entities.CreateEntityUninitialized(null, parent, new RigidTransform3(position, Quaternion.Identity));
        var motor = entities.AddComponent<SpatialMotorComponent>(uid);
        var body = entities.AddComponent<SpatialBodyComponent>(uid);
        physics.ConfigureInitialMotor((uid, motor), SpatialMotorSettings.Default, intent);
        physics.ConfigureInitialBody((uid, body), new SpatialBodyDefinition(SpatialMotionOwner.Motor,
            new[] { new SpatialColliderDefinition(new SpatialCapsuleShape(0.3f, 1), RigidTransform3.Identity) }, mass: 80,
            material: SpatialContactMaterial.Default with { Friction = 0 }));
        entities.InitializeAndStartEntity(uid, entities.GetComponent<TransformComponent>(parent).MapID);
        return (uid, motor);
    }

    internal static EntityUid Floor(IEntityManager entities, EntityUid world)
    {
        var frame = entities.CreateEntityUninitialized(null, world, RigidTransform3.Identity);
        var layers = entities.AddComponent<SpatialFrameComponent>(frame);
        var layer = new LayerId(1);
        entities.System<SharedSpatialSystem>().ConfigureInitialLayers((frame, layers),
            new Dictionary<LayerId, LayerDefinition> { [layer] = new(0, "Floor") });
        var geometry = entities.AddComponent<SpatialGeometryComponent>(frame);
        entities.System<SharedGeometrySystem>().ConfigureInitialGeometry((frame, geometry), Array.Empty<BoundarySegment>(),
            new[] { new SlabSurface(new SpatialSurfaceReference(frame, new SurfaceId(1)), layer, new Box2(-20, -20, 20, 20), 0, 0.2f) });
        entities.InitializeAndStartEntity(frame, entities.GetComponent<MapComponent>(world).MapId);
        return frame;
    }

    [Test]
    public void MotorLandsOnceAfterCanonicalCommitAndLosesDeletedSupport()
    {
        var sim = RobustServerSimulation.NewSimulation()
            .RegisterEntitySystems(f => f.LoadExtraSystemType<MotorObserver>()).InitializeInstance();
        var entities = sim.Resolve<IEntityManager>();
        var physics = entities.System<SharedSpatialPhysicsSystem>();
        var world = entities.System<SharedSpatialSystem>().CreateWorld();
        physics.EnablePhysics(world);
        var floor = Floor(entities, world);
        var motor = SpawnMotor(entities, world, Vector3.UnitZ * 4);
        var observer = entities.System<MotorObserver>();
        observer.OnLanded = (uid, ev) =>
        {
            Assert.That(uid, Is.EqualTo(motor.Owner));
            Assert.That(entities.System<SharedTransformSystem>().GetWorldPose(uid).Position.Z, Is.EqualTo(0.8).Within(0.02));
            Assert.That(ev.RelativeVelocity.Z, Is.LessThan(-5));
            Assert.That(motor.Comp.Support, Is.Not.Null);
        };
        for (var tick = 0; tick < 240; tick++)
            physics.Update(1f / 60);
        Assert.That(observer.Landings, Is.EqualTo(1));
        Assert.That(motor.Comp.Support!.Value.Source.Surface, Is.EqualTo(new SpatialSurfaceReference(floor, new SurfaceId(1))));
        physics.SetMotorIntent(motor, new SpatialMotorIntent(Vector3.UnitX * 2, SpatialMotorMode.Walking));
        for (var tick = 0; tick < 60; tick++)
            physics.Update(1f / 60);
        Assert.That(entities.System<SharedTransformSystem>().GetWorldPose(motor).Position.X, Is.GreaterThan(1.8));
        Assert.That(observer.Landings, Is.EqualTo(1));
        entities.DeleteEntity(floor);
        physics.Update(1f / 60);
        Assert.That(motor.Comp.Support, Is.Null);
        Assert.That(observer.Losses, Is.EqualTo(1));
        Assert.That(entities.GetComponent<SpatialBodyComponent>(motor).LinearVelocity.Z, Is.LessThan(0));
    }

    [Test]
    public void ResizeRejectsCeilingAndPreservesFootRevisionAndContacts()
    {
        var sim = RobustServerSimulation.NewSimulation().InitializeInstance();
        var entities = sim.Resolve<IEntityManager>();
        var physics = entities.System<SharedSpatialPhysicsSystem>();
        var world = entities.System<SharedSpatialSystem>().CreateWorld();
        physics.EnablePhysics(world);
        Floor(entities, world);
        var motor = SpawnMotor(entities, world, Vector3.UnitZ * 0.8f);
        var body = entities.GetComponent<SpatialBodyComponent>(motor);
        for (var i = 0; i < 30; i++) physics.Update(1f / 60);
        var pose = entities.System<SharedTransformSystem>().GetWorldPose(motor);
        Assert.That(physics.TryResizeMotor(motor, 0.3f, 0.5f, 0), Is.True);
        Assert.That(entities.System<SharedTransformSystem>().GetWorldPose(motor), Is.EqualTo(pose));
        Assert.That(body.Definition.Colliders[0].Offset.Z, Is.EqualTo(-0.25f).Within(1e-6));
        var ceiling = entities.CreateEntityUninitialized(null, world, new RigidTransform3(Vector3.UnitZ * 1.8f, Quaternion.Identity));
        var ceilingBody = entities.AddComponent<SpatialBodyComponent>(ceiling);
        physics.ConfigureInitialBody((ceiling, ceilingBody), new SpatialBodyDefinition(SpatialMotionOwner.Static,
            new[] { new SpatialColliderDefinition(new SpatialBoxShape(new Vector3(10, 10, 1)), RigidTransform3.Identity) }));
        entities.InitializeAndStartEntity(ceiling, entities.GetComponent<MapComponent>(world).MapId);
        var definition = body.Definition;
        Assert.That(physics.TryResizeMotor(motor, 0.3f, 1, 1), Is.False);
        Assert.That(body.Definition, Is.SameAs(definition));
        Assert.That(body.DefinitionRevision, Is.EqualTo(1));
        entities.DeleteEntity(ceiling);
        Assert.That(physics.TryResizeMotor(motor, 0.3f, 1, 1), Is.True);
        Assert.That(body.DefinitionRevision, Is.EqualTo(2));
        Assert.Throws<InvalidOperationException>(() => physics.TryResizeMotor(motor, 0.3f, 0.5f, 1));
    }

    [Test]
    public void MotorContainerPauseAndOwnerChangeKeepOnePhysicalOwner()
    {
        var sim = RobustServerSimulation.NewSimulation().InitializeInstance();
        var entities = sim.Resolve<IEntityManager>();
        var physics = entities.System<SharedSpatialPhysicsSystem>();
        var world = entities.System<SharedSpatialSystem>().CreateWorld();
        physics.EnablePhysics(world, new SpatialPhysicsSettings(Vector3.Zero, 1, 8));
        var motor = SpawnMotor(entities, world, Vector3.Zero, new SpatialMotorIntent(Vector3.UnitX, SpatialMotorMode.Free));
        var body = entities.GetComponent<SpatialBodyComponent>(motor);
        var holder = entities.SpawnAttachedTo(null, world, RigidTransform3.Identity);
        var containers = entities.System<SharedContainerSystem>();
        var container = containers.EnsureContainer<Container>(holder, "storage");
        Assert.That(containers.Insert(motor.Owner, container), Is.True);
        physics.Update(1f / 30);
        Assert.That(physics.GetBodyCount(world), Is.Zero);
        Assert.Throws<InvalidOperationException>(() => physics.TryResizeMotor(motor, 0.3f, 0.5f, 0));
        Assert.That(containers.Remove(motor.Owner, container), Is.True);
        entities.System<MetaDataSystem>().SetEntityPaused(motor, true);
        physics.Update(1f / 30);
        Assert.That(entities.System<SharedTransformSystem>().GetWorldPose(motor).Position, Is.EqualTo(Vector3.Zero));
        entities.System<MetaDataSystem>().SetEntityPaused(motor, false);
        physics.Update(1f / 30);
        Assert.That(body.LinearVelocity.X, Is.GreaterThan(0));
        physics.ReconfigureBody((motor.Owner, body), new SpatialBodyDefinition(SpatialMotionOwner.DynamicBody,
            body.Definition.Colliders, mass: 80), 0, Vector3.Zero, Vector3.Zero);
        physics.Update(1f / 30);
        Assert.That(body.LinearVelocity, Is.EqualTo(Vector3.Zero));
        Assert.That(physics.GetBodyCount(world), Is.EqualTo(1));
        entities.DeleteEntity(world);
        Assert.DoesNotThrow(() => physics.Update(1f / 30));
    }

    [Test]
    public void LandingHandlerCanDeleteWorldAfterSupportCommit()
    {
        var sim = RobustServerSimulation.NewSimulation()
            .RegisterEntitySystems(f => f.LoadExtraSystemType<MotorObserver>()).InitializeInstance();
        var entities = sim.Resolve<IEntityManager>();
        var physics = entities.System<SharedSpatialPhysicsSystem>();
        var world = entities.System<SharedSpatialSystem>().CreateWorld();
        physics.EnablePhysics(world);
        Floor(entities, world);
        var motor = SpawnMotor(entities, world, Vector3.UnitZ * 0.8f);
        entities.System<MotorObserver>().OnLanded = (uid, _) =>
        {
            Assert.That(uid, Is.EqualTo(motor.Owner));
            Assert.That(motor.Comp.Support, Is.Not.Null);
            entities.DeleteEntity(world);
        };
        Assert.DoesNotThrow(() => physics.Update(1f / 60));
        Assert.That(entities.EntityExists(world), Is.False);
        Assert.DoesNotThrow(() => physics.Update(1f / 60));
    }

    [Reflect(false)]
    private sealed class MotorObserver : EntitySystem
    {
        public int Landings;
        public int Losses;
        public Action<EntityUid, SpatialMotorLanded>? OnLanded;
        public override void Initialize()
        {
            SubscribeLocalEvent<SpatialMotorComponent, SpatialMotorLanded>(OnLanding);
            SubscribeLocalEvent<SpatialMotorComponent, SpatialMotorSupportChanged>(OnSupport);
        }
        private void OnLanding(EntityUid uid, SpatialMotorComponent component, ref SpatialMotorLanded ev)
        {
            Landings++;
            OnLanded?.Invoke(uid, ev);
        }
        private void OnSupport(EntityUid uid, SpatialMotorComponent component, ref SpatialMotorSupportChanged ev)
        {
            if (ev.Current == null) Losses++;
        }
    }
}
