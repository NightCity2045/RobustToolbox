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
public sealed class SpatialImpulseTests
{
    [Test]
    public void ImpulsePublishesCanonicalVelocityBeforeItsSingleAuthoritativeEvent()
    {
        var sim = RobustServerSimulation.NewSimulation()
            .RegisterEntitySystems(f => f.LoadExtraSystemType<ImpulseObserver>()).InitializeInstance();
        var entities = sim.Resolve<IEntityManager>();
        var physics = entities.System<SharedSpatialPhysicsSystem>();
        var world = entities.System<SharedSpatialSystem>().CreateWorld();
        physics.EnablePhysics(world, new SpatialPhysicsSettings(Vector3.Zero, 1, 8));
        var uid = SpatialPhysicsTests.SpawnBody(entities, world, Vector3.Zero);
        Entity<SpatialBodyComponent> body = (uid, entities.GetComponent<SpatialBodyComponent>(uid));
        var observer = entities.System<ImpulseObserver>();
        physics.ApplyImpulse(body, Vector3.UnitY * 2, Vector3.Zero);
        Assert.That(observer.Count, Is.EqualTo(1));
        Assert.That(body.Comp.LinearVelocity, Is.EqualTo(Vector3.UnitY * 2));
        Assert.That(physics.GetVelocityAtWorldPoint(body, Vector3.UnitX), Is.EqualTo(Vector3.UnitY * 2));
        physics.ApplyImpulse(body, Vector3.Zero, Vector3.Zero);
        Assert.Throws<ArgumentException>(() => physics.ApplyImpulse(body, new Vector3(float.NaN), Vector3.Zero));
        Assert.That(observer.Count, Is.EqualTo(1));
        physics.Update(1f / 30);
        Assert.That(entities.System<SharedTransformSystem>().GetWorldPose(uid).Position.Y, Is.EqualTo(2f / 30).Within(1e-5));
        Assert.That(observer.Count, Is.EqualTo(1));
        entities.System<MetaDataSystem>().SetEntityPaused(uid, true);
        Assert.Throws<InvalidOperationException>(() => physics.ApplyImpulse(body, Vector3.UnitY, Vector3.Zero));
        entities.System<MetaDataSystem>().SetEntityPaused(uid, false);
        var holder = entities.SpawnAttachedTo(null, world, RigidTransform3.Identity);
        var containers = entities.System<SharedContainerSystem>();
        var container = containers.EnsureContainer<Container>(holder, "storage");
        Assert.That(containers.Insert(uid, container), Is.True);
        Assert.Throws<InvalidOperationException>(() => physics.ApplyImpulse(body, Vector3.UnitY, Vector3.Zero));
        Assert.That(observer.Count, Is.EqualTo(1));
    }

    [Test]
    public void PointVelocityIncludesAngularTermAboutEntityOrigin()
    {
        var sim = RobustServerSimulation.NewSimulation().InitializeInstance();
        var entities = sim.Resolve<IEntityManager>();
        var physics = entities.System<SharedSpatialPhysicsSystem>();
        var world = entities.System<SharedSpatialSystem>().CreateWorld();
        physics.EnablePhysics(world);
        var uid = SpatialPhysicsTests.SpawnBody(entities, world, new Vector3(3, 4, 5));
        Entity<SpatialBodyComponent> body = (uid, entities.GetComponent<SpatialBodyComponent>(uid));
        physics.SetVelocity(body, Vector3.UnitX, new Vector3(0, 0, 2));
        Assert.That(physics.GetVelocityAtWorldPoint(body, new Vector3(3, 7, 5)), Is.EqualTo(new Vector3(-5, 0, 0)));
    }

    [Reflect(false)]
    private sealed class ImpulseObserver : EntitySystem
    {
        public int Count;
        public override void Initialize() => SubscribeLocalEvent<SpatialBodyComponent, SpatialImpulseApplied>(OnImpulse);
        private void OnImpulse(EntityUid uid, SpatialBodyComponent component, ref SpatialImpulseApplied ev)
        {
            Assert.That(component.LinearVelocity, Is.EqualTo(ev.LinearVelocity));
            Assert.That(component.AngularVelocity, Is.EqualTo(ev.AngularVelocity));
            Count++;
        }
    }
}
