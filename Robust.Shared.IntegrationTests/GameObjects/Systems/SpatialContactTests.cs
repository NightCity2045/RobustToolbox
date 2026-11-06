using System.Numerics;
using NUnit.Framework;
using Robust.Shared.GameObjects;
using Robust.Shared.Map.Components;
using Robust.Shared.Maths;
using Robust.Shared.Reflection;
using Robust.Shared.Spatial;
using Robust.Shared.Spatial.Physics;
using Robust.UnitTesting.Server;

namespace Robust.UnitTesting.Shared.GameObjects.Systems;

[TestFixture]
public sealed class SpatialContactTests
{
    [Test]
    public void SurfaceSeamsSleepAndPauseDoNotDuplicateGameplayContacts()
    {
        var sim = RobustServerSimulation.NewSimulation()
            .RegisterEntitySystems(f => f.LoadExtraSystemType<ContactObserver>()).InitializeInstance();
        var entities = sim.Resolve<IEntityManager>();
        var (world, floor, body, surface) = CreateScene(entities);
        var physics = entities.System<SharedSpatialPhysicsSystem>();
        var observer = entities.System<ContactObserver>();
        physics.Update(1f / 60);
        Assert.That(observer.Started.GetValueOrDefault(body), Is.EqualTo(1));
        var contact = observer.Last[body];
        Assert.That(contact.Other.Surface, Is.EqualTo(surface));
        Assert.That(contact.Other.Revision, Is.EqualTo(4));
        Assert.That(contact.Normal.Z, Is.GreaterThan(0.99));
        Assert.That(contact.Point.Z, Is.EqualTo(1.75f).Within(0.001));
        for (var i = 0; i < 300; i++)
            physics.Update(1f / 60);
        Assert.That(observer.Started.GetValueOrDefault(body), Is.EqualTo(1));
        Assert.That(observer.Ended.GetValueOrDefault(body), Is.Zero);
        entities.System<MetaDataSystem>().SetEntityPaused(body, true);
        for (var i = 0; i < 30; i++)
            physics.Update(1f / 60);
        entities.System<MetaDataSystem>().SetEntityPaused(body, false);
        physics.Update(1f / 60);
        Assert.That(observer.Started.GetValueOrDefault(body), Is.EqualTo(1));
        Assert.That(observer.Ended.GetValueOrDefault(body), Is.Zero);
        entities.DeleteEntity(floor);
        physics.Update(1f / 60);
        Assert.That(observer.Ended.GetValueOrDefault(body), Is.EqualTo(1));
        Assert.That(entities.GetComponent<SpatialBodyComponent>(body).LinearVelocity.Z, Is.LessThan(0));
        entities.DeleteEntity(world);
        Assert.DoesNotThrow(() => physics.Update(1f / 60));
    }

    [Test]
    public void ContactHandlersCanDeleteTheWorldAfterCanonicalStateCommit()
    {
        var sim = RobustServerSimulation.NewSimulation()
            .RegisterEntitySystems(f => f.LoadExtraSystemType<ContactObserver>()).InitializeInstance();
        var entities = sim.Resolve<IEntityManager>();
        var (world, _, body, _) = CreateScene(entities);
        var physics = entities.System<SharedSpatialPhysicsSystem>();
        physics.SetVelocity((body, entities.GetComponent<SpatialBodyComponent>(body)), -Vector3.UnitZ, Vector3.Zero);
        entities.System<ContactObserver>().OnStarted = (uid, contact) =>
        {
            if (uid != body)
                return;
            Assert.That(contact.RelativeVelocity.Z, Is.LessThan(-0.9));
            Assert.That(entities.GetComponent<SpatialBodyComponent>(body).LinearVelocity.Z, Is.GreaterThan(-0.2));
            entities.DeleteEntity(world);
        };
        Assert.DoesNotThrow(() => physics.Update(1f / 60));
        Assert.That(entities.EntityExists(world), Is.False);
        Assert.DoesNotThrow(() => physics.Update(1f / 60));
    }

    private static (EntityUid World, EntityUid Floor, EntityUid Body, SpatialSurfaceReference Surface) CreateScene(IEntityManager entities)
    {
        var world = entities.System<SharedSpatialSystem>().CreateWorld();
        entities.System<SharedSpatialPhysicsSystem>().EnablePhysics(world);
        var frame = SpatialGeometryTests.CreateUninitializedFrame(entities, world);
        var surface = new SpatialSurfaceReference(frame, new SurfaceId(8));
        var slab = new SlabSurface(surface, new LayerId(5), new Box2(-2, -2, 2, 2), 0, 0.2f,
            openings: new[] { new Box2(0, 0, 1, 1) }, stateRevision: 4);
        entities.System<SharedGeometrySystem>().ConfigureInitialGeometry(
            (frame, entities.GetComponent<SpatialGeometryComponent>(frame)), Array.Empty<BoundarySegment>(), new[] { slab }, 4);
        entities.InitializeAndStartEntity(frame, entities.GetComponent<MapComponent>(world).MapId);
        // This point lies on the seam of two convex parts of the same slab.
        var body = SpatialPhysicsTests.SpawnBody(entities, world, new Vector3(0, -1, 2));
        return (world, frame, body, surface);
    }

    [Reflect(false)]
    private sealed class ContactObserver : EntitySystem
    {
        public readonly Dictionary<EntityUid, int> Started = new();
        public readonly Dictionary<EntityUid, int> Ended = new();
        public readonly Dictionary<EntityUid, SpatialContact> Last = new();
        public Action<EntityUid, SpatialContact>? OnStarted;
        public override void Initialize()
        {
            SubscribeLocalEvent<TransformComponent, SpatialContactStarted>(Start);
            SubscribeLocalEvent<TransformComponent, SpatialContactEnded>(End);
        }
        private void Start(EntityUid uid, TransformComponent component, ref SpatialContactStarted ev)
        {
            Started[uid] = Started.GetValueOrDefault(uid) + 1;
            Last[uid] = ev.Contact;
            OnStarted?.Invoke(uid, ev.Contact);
        }
        private void End(EntityUid uid, TransformComponent component, ref SpatialContactEnded ev)
        {
            Ended[uid] = Ended.GetValueOrDefault(uid) + 1;
            Last[uid] = ev.Contact;
        }
    }
}
