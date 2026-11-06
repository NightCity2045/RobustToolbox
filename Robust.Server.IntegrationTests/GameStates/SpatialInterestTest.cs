using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Threading.Tasks;
using NUnit.Framework;
using Robust.Server.GameObjects;
using Robust.Server.Spatial;
using Robust.Shared;
using Robust.Shared.Containers;
using Robust.Shared.GameObjects;
using Robust.Shared.Maths;
using Robust.Shared.Spatial;
using Robust.Shared.Spatial.Physics;
using Robust.UnitTesting.Server;

namespace Robust.UnitTesting.Server.GameStates;

[TestFixture]
public sealed class SpatialInterestTest : RobustIntegrationTest
{
    [Test]
    public async Task NetworkInterestUsesHeightParentMotionAndContainerVisibility()
    {
        await using var pair = await StartConnectedPair();
        var (client, server) = pair;
        await RunTicksSync(server, client, 10);
        var entities = server.EntMan;
        EntityUid world = default, parent = default, child = default, high = default, hidden = default, item = default, viewer = default;
        NetEntity childNet = default, highNet = default, hiddenNet = default, itemNet = default;
        await server.WaitPost(() =>
        {
            server.CfgMan.SetCVar(CVars.NetPVS, true);
            world = server.System<SharedSpatialSystem>().CreateWorld();
            viewer = entities.SpawnAttachedTo(null, world, RigidTransform3.Identity);
            parent = entities.SpawnAttachedTo(null, world, new RigidTransform3(new Vector3(100, 0, 0), Quaternion.Identity));
            child = entities.SpawnAttachedTo(null, parent, new RigidTransform3(new Vector3(-99, 0, 0), Quaternion.Identity));
            high = entities.SpawnAttachedTo(null, world, new RigidTransform3(new Vector3(0, 0, 100), Quaternion.Identity));
            hidden = entities.SpawnAttachedTo(null, world, RigidTransform3.Identity);
            var visibility = entities.AddComponent<VisibilityComponent>(hidden);
            server.System<VisibilitySystem>().AddLayer((hidden, visibility), 2);
            var box = entities.SpawnAttachedTo(null, world, RigidTransform3.Identity);
            item = entities.SpawnAttachedTo(null, world, RigidTransform3.Identity);
            var containers = server.System<SharedContainerSystem>();
            var container = containers.EnsureContainer<Container>(box, "secret");
            Assert.That(containers.Insert(item, container), Is.True);
            childNet = entities.GetNetEntity(child);
            highNet = entities.GetNetEntity(high);
            hiddenNet = entities.GetNetEntity(hidden);
            itemNet = entities.GetNetEntity(item);
            var session = server.PlayerMan.Sessions.First();
            server.PlayerMan.SetAttachedEntity(session, viewer);
            server.PlayerMan.JoinGame(session);
        });
        await RunTicksSync(server, client, 15);
        await client.WaitAssertion(() =>
        {
            Assert.That(client.EntMan.TryGetEntity(childNet, out var remote), Is.True);
            var transform = client.EntMan.GetComponent<TransformComponent>(remote!.Value);
            Assert.That(transform.ParentUid, Is.EqualTo(client.EntMan.GetEntity(entities.GetNetEntity(parent))));
            Assert.That(client.System<SharedTransformSystem>().GetWorldPose(remote.Value).Position.X, Is.EqualTo(1).Within(0.001));
            Assert.That(client.EntMan.TryGetEntity(highNet, out _), Is.False, "Same XY at a distant Z must not enter interest.");
            Assert.That(client.EntMan.TryGetEntity(hiddenNet, out _), Is.False, "Visibility masks still apply.");
            // Container states may create an unresolved NetEntity placeholder. It
            // must not have received metadata or the hidden object's components.
            Assert.That(client.EntMan.TryGetEntityData(itemNet, out _, out var meta) && meta.EntityLifeStage >= EntityLifeStage.Initialized,
                Is.False, "Closed container contents must not receive complete entity state.");
        });

        await server.WaitPost(() => server.System<SharedTransformSystem>().SetWorldPose(parent,
            new RigidTransform3(new Vector3(100, 0, 100), Quaternion.Identity)));
        await RunTicksSync(server, client, 15);
        await client.WaitAssertion(() =>
        {
            var remote = client.EntMan.GetEntity(childNet);
            Assert.That(client.EntMan.GetComponent<MetaDataComponent>(remote).Flags.HasFlag(MetaDataFlags.Detached), Is.True);
        });
        await server.WaitPost(() => server.System<SharedTransformSystem>().SetWorldPose(viewer,
            new RigidTransform3(new Vector3(0, 0, 100), Quaternion.Identity)));
        await RunTicksSync(server, client, 15);
        await client.WaitAssertion(() =>
        {
            var remote = client.EntMan.GetEntity(childNet);
            Assert.That(client.EntMan.GetComponent<MetaDataComponent>(remote).Flags.HasFlag(MetaDataFlags.Detached), Is.False);
            Assert.That(client.System<SharedTransformSystem>().GetWorldPose(remote).Position.Z, Is.EqualTo(100).Within(0.001));
            Assert.That(client.EntMan.TryGetEntity(highNet, out _), Is.True);
        });
    }

    [Test]
    public void BoundsFollowRotationWorldTransferReconfigurationAndDeletion()
    {
        var sim = RobustServerSimulation.NewSimulation().InitializeInstance();
        var entities = sim.Resolve<IEntityManager>();
        var interest = entities.System<SpatialInterestSystem>();
        var spatial = entities.System<SharedSpatialSystem>();
        var transforms = entities.System<SharedTransformSystem>();
        var physics = entities.System<SharedSpatialPhysicsSystem>();
        var world = spatial.CreateWorld();
        var second = spatial.CreateWorld();
        physics.EnablePhysics(world);
        physics.EnablePhysics(second);
        var parent = entities.SpawnAttachedTo(null, world, RigidTransform3.Identity);
        var uid = entities.CreateEntityUninitialized(null, parent, new RigidTransform3(new Vector3(10, 0, 0), Quaternion.Identity));
        var body = entities.AddComponent<SpatialBodyComponent>(uid);
        physics.ConfigureInitialBody((uid, body), Definition(2));
        entities.InitializeAndStartEntity(uid);
        var found = new HashSet<EntityUid>();
        var surfaces = new HashSet<SpatialSurfaceReference>();
        var atX = new Box3(new Vector3(8, -1, -1), new Vector3(12, 1, 1));
        var atZ = new Box3(new Vector3(-1, -1, 8), new Vector3(1, 1, 12));
        interest.Collect(world, atX, _ => true, found, surfaces);
        Assert.That(found, Does.Contain(uid));
        transforms.SetWorldPose(parent, new RigidTransform3(Vector3.Zero, Quaternion.CreateFromAxisAngle(Vector3.UnitY, -MathF.PI / 2)));
        found.Clear();
        interest.Collect(world, atZ, _ => true, found, surfaces);
        Assert.That(found, Does.Contain(uid));
        found.Clear();
        interest.Collect(world, atX, _ => true, found, surfaces);
        Assert.That(found, Does.Not.Contain(uid));
        transforms.SetParent(parent, second);
        found.Clear();
        interest.Collect(world, atZ, _ => true, found, surfaces);
        Assert.That(found, Does.Not.Contain(uid));
        interest.Collect(second, atZ, _ => true, found, surfaces);
        Assert.That(found, Does.Contain(uid));
        var edge = new Box3(new Vector3(3, -1, 9), new Vector3(4, 1, 11));
        found.Clear();
        interest.Collect(second, edge, _ => true, found, surfaces);
        Assert.That(found, Does.Not.Contain(uid));
        physics.ReconfigureBody((uid, body), Definition(10), body.DefinitionRevision, Vector3.Zero, Vector3.Zero);
        interest.Collect(second, edge, _ => true, found, surfaces);
        Assert.That(found, Does.Contain(uid));
        entities.DeleteEntity(uid);
        found.Clear();
        interest.Collect(second, atZ, _ => true, found, surfaces);
        Assert.That(found, Does.Not.Contain(uid));

        static SpatialBodyDefinition Definition(float size) => new(SpatialMotionOwner.Static,
            new[] { new SpatialColliderDefinition(new SpatialBoxShape(new Vector3(size)), RigidTransform3.Identity) });
    }

    [Test]
    public void GeometryInterestUsesSurfaceBoundsAndCommittedRevisions()
    {
        var sim = RobustServerSimulation.NewSimulation().InitializeInstance();
        var entities = sim.Resolve<IEntityManager>();
        var spatial = entities.System<SharedSpatialSystem>();
        var geometry = entities.System<SharedGeometrySystem>();
        var interest = entities.System<SpatialInterestSystem>();
        var world = spatial.CreateWorld();
        var source = entities.SpawnAttachedTo(null, world, RigidTransform3.Identity);
        var frame = entities.CreateEntityUninitialized(null, world,
            new RigidTransform3(new Vector3(500, 0, 0), Quaternion.Identity));
        var layer = new LayerId(1);
        var frameComp = entities.AddComponent<SpatialFrameComponent>(frame);
        spatial.ConfigureInitialLayers((frame, frameComp), new Dictionary<LayerId, LayerDefinition> { [layer] = new(0, "Floor") });
        var geo = entities.AddComponent<SpatialGeometryComponent>(frame);
        var surface = new SpatialSurfaceReference(frame, new SurfaceId(1));
        geometry.ConfigureInitialGeometry((frame, geo), Array.Empty<BoundarySegment>(),
            new[] { new SlabSurface(surface, layer, new Box2(-501, -1, -499, 1), 0, 0.2f, entity: source) });
        entities.InitializeAndStartEntity(frame);
        var query = new Box3(new Vector3(-2), new Vector3(2));
        var found = new HashSet<EntityUid>();
        var surfaces = new HashSet<SpatialSurfaceReference>();
        interest.Collect(world, query, _ => true, found, surfaces);
        Assert.That(found, Does.Contain(frame), "A surface can be near the view even when its frame origin is far away.");
        Assert.That(surfaces, Does.Contain(surface));
        surfaces.Clear();
        found.Clear();
        interest.Collect(world, query, uid => uid != source, found, surfaces);
        Assert.That(surfaces, Is.Empty, "Geometry cannot bypass a denied source entity.");
        using var transaction = geometry.PrepareTransaction(new[]
        {
            new SpatialGeometryEdit(geometry.CaptureSnapshot(frame),
                layers: new Dictionary<LayerId, LayerDefinition> { [layer] = new(100, "Floor") })
        });
        geometry.QueueTransaction(transaction);
        entities.System<SharedSpatialPhysicsSystem>().Update(1f / 60);
        Assert.That(transaction.Status, Is.EqualTo(WorldEditStatus.Committed));
        surfaces.Clear();
        interest.Collect(world, query, _ => true, found, surfaces);
        Assert.That(surfaces, Is.Empty);
        interest.Collect(world, new Box3(new Vector3(-2, -2, 98), new Vector3(2, 2, 102)), _ => true, found, surfaces);
        Assert.That(surfaces, Does.Contain(surface));
        entities.RemoveComponent<SpatialGeometryComponent>(frame);
        surfaces.Clear();
        interest.Collect(world, new Box3(new Vector3(-2, -2, 98), new Vector3(2, 2, 102)), _ => true, found, surfaces);
        Assert.That(surfaces, Is.Empty);
    }

    [Test]
    public async Task AuthorizedRemoteViewIsScopedToItsWorldAndRevocationLeavesInterest()
    {
        await using var pair = await StartConnectedPair();
        var (client, server) = pair;
        await RunTicksSync(server, client, 10);
        EntityUid camera = default;
        NetEntity target = default, far = default;
        await server.WaitPost(() =>
        {
            server.CfgMan.SetCVar(CVars.NetPVS, true);
            var a = server.System<SharedSpatialSystem>().CreateWorld();
            var b = server.System<SharedSpatialSystem>().CreateWorld();
            var viewer = server.EntMan.SpawnAttachedTo(null, a, RigidTransform3.Identity);
            camera = server.EntMan.SpawnAttachedTo(null, b, new RigidTransform3(new Vector3(0, 0, 60), Quaternion.Identity));
            target = server.EntMan.GetNetEntity(server.EntMan.SpawnAttachedTo(null, b,
                new RigidTransform3(new Vector3(1, 0, 60), Quaternion.Identity)));
            far = server.EntMan.GetNetEntity(server.EntMan.SpawnAttachedTo(null, b, RigidTransform3.Identity));
            var session = server.PlayerMan.Sessions.First();
            server.PlayerMan.SetAttachedEntity(session, viewer);
            server.PlayerMan.JoinGame(session);
        });
        await RunTicksSync(server, client, 10);
        await client.WaitAssertion(() => Assert.That(client.EntMan.TryGetEntity(target, out _), Is.False));
        await server.WaitPost(() => server.System<ViewSubscriberSystem>().AddViewSubscriber(camera, server.PlayerMan.Sessions.First()));
        await RunTicksSync(server, client, 10);
        await client.WaitAssertion(() =>
        {
            Assert.That(client.EntMan.TryGetEntity(target, out _), Is.True);
            Assert.That(client.EntMan.TryGetEntity(far, out _), Is.False, "A remote camera must not expose its entire world.");
        });
        await server.WaitPost(() => server.System<ViewSubscriberSystem>().RemoveViewSubscriber(camera, server.PlayerMan.Sessions.First()));
        await RunTicksSync(server, client, 15);
        await client.WaitAssertion(() => Assert.That(
            client.EntMan.GetComponent<MetaDataComponent>(client.EntMan.GetEntity(target)).Flags.HasFlag(MetaDataFlags.Detached), Is.True));
    }

    [Test]
    public async Task VehicleCameraPreloadsAlongWorldVelocityWithoutSendingDistantOrRearEntities()
    {
        await using var pair = await StartConnectedPair();
        var (client, server) = pair;
        await RunTicksSync(server, client, 10);
        EntityUid vehicle = default;
        NetEntity ahead = default, rear = default, distant = default;
        await server.WaitPost(() =>
        {
            server.CfgMan.SetCVar(CVars.NetPVS, true);
            server.CfgMan.SetCVar(CVars.NetTickrate, 60);
            server.CfgMan.SetCVar(CVars.NetMaxUpdateRange, 24f);
            server.CfgMan.SetCVar(CVars.NetPvsPriorityRange, 24f);
            server.CfgMan.SetCVar(CVars.NetSpatialPreloadSeconds, 0.5f);
            server.CfgMan.SetCVar(CVars.NetSpatialPreloadDistance, 64f);
            var world = server.System<SharedSpatialSystem>().CreateWorld();
            var physics = server.System<SharedSpatialPhysicsSystem>();
            physics.EnablePhysics(world, new SpatialPhysicsSettings(Vector3.Zero, 1, 8));
            vehicle = server.EntMan.CreateEntityUninitialized(null, world, RigidTransform3.Identity);
            var body = server.EntMan.AddComponent<SpatialBodyComponent>(vehicle);
            physics.ConfigureInitialBody((vehicle, body), new SpatialBodyDefinition(SpatialMotionOwner.DynamicBody,
                new[] { new SpatialColliderDefinition(new SpatialBoxShape(Vector3.One), RigidTransform3.Identity) }),
                new Vector3(120, 0, 0));
            server.EntMan.InitializeAndStartEntity(vehicle);
            var camera = server.EntMan.SpawnAttachedTo(null, vehicle, RigidTransform3.Identity);
            ahead = Spawn(50);
            rear = Spawn(-50);
            distant = Spawn(200);
            var session = server.PlayerMan.Sessions.First();
            server.PlayerMan.SetAttachedEntity(session, camera);
            server.PlayerMan.JoinGame(session);
            NetEntity Spawn(float x) => server.EntMan.GetNetEntity(server.EntMan.SpawnAttachedTo(null, world,
                new RigidTransform3(new Vector3(x, 0, 0), Quaternion.Identity)));
        });
        // Keep the measured view near the origin while its canonical velocity is
        // nonzero. Otherwise waiting for the client could move the view into the
        // ordinary range and accidentally test proximity instead of preloading.
        for (var i = 0; i < 15; i++)
        {
            await server.WaitPost(() => server.System<SharedTransformSystem>().SetWorldPose(vehicle, RigidTransform3.Identity));
            await RunTicksSync(server, client, 1);
        }
        await server.WaitAssertion(() => Assert.That(server.System<SharedTransformSystem>().GetWorldPose(vehicle).Position.X,
            Is.LessThan(10)));
        await client.WaitAssertion(() =>
        {
            Assert.That(client.EntMan.TryGetEntity(ahead, out _), Is.True);
            Assert.That(client.EntMan.TryGetEntity(rear, out _), Is.False);
            Assert.That(client.EntMan.TryGetEntity(distant, out _), Is.False);
        });
    }
}
