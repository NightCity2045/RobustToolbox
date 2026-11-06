using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Threading.Tasks;
using NUnit.Framework;
using Robust.Client.Spatial;
using Robust.Shared;
using Robust.Shared.Containers;
using Robust.Shared.GameObjects;
using Robust.Shared.Maths;
using Robust.Shared.Spatial;
using Robust.Shared.Spatial.Physics;

namespace Robust.UnitTesting.Server.GameStates;

[TestFixture]
public sealed class SpatialCollisionViewTest : RobustIntegrationTest
{
    [TestCase(true)]
    [TestCase(false)]
    public async Task QueriesRequireCoherentAuthorizedGeometryAndExpireDuringReplacement(bool pvs)
    {
        await using var pair = await StartConnectedPair();
        var (client, server) = pair;
        await RunTicksSync(server, client, 10);
        EntityUid world = default, frame = default, body = default, viewer = default;
        NetEntity worldNet = default, frameNet = default, bodyNet = default;
        var layer = new LayerId(1);
        await server.WaitPost(() =>
        {
            server.CfgMan.SetCVar(CVars.NetPVS, pvs);
            server.CfgMan.SetCVar(CVars.NetSpatialGeometryBytes, 65536);
            world = server.System<SharedSpatialSystem>().CreateWorld();
            server.System<SharedSpatialPhysicsSystem>().EnablePhysics(world, new SpatialPhysicsSettings(Vector3.Zero, 1, 8));
            frame = server.EntMan.CreateEntityUninitialized(null, world, RigidTransform3.Identity);
            var frames = server.EntMan.AddComponent<SpatialFrameComponent>(frame);
            server.System<SharedSpatialSystem>().ConfigureInitialLayers((frame, frames),
                new Dictionary<LayerId, LayerDefinition> { [layer] = new(0, "Deck") });
            var geometry = server.EntMan.AddComponent<SpatialGeometryComponent>(frame);
            server.System<SharedGeometrySystem>().ConfigureInitialGeometry((frame, geometry), Array.Empty<BoundarySegment>(),
                new[] { new SlabSurface(new SpatialSurfaceReference(frame, new SurfaceId(1)), layer,
                    new Box2(-2, -2, 2, 2), 0, 0.2f, materialProfile: "metal") });
            server.EntMan.InitializeAndStartEntity(frame);
            body = server.EntMan.CreateEntityUninitialized(null, world,
                new RigidTransform3(new Vector3(2, 0, 1), Quaternion.Identity));
            var definition = server.EntMan.AddComponent<SpatialBodyComponent>(body);
            server.System<SharedSpatialPhysicsSystem>().ConfigureInitialBody((body, definition),
                new SpatialBodyDefinition(SpatialMotionOwner.Static,
                    new[] { new SpatialColliderDefinition(new SpatialBoxShape(Vector3.One), RigidTransform3.Identity) }, collisionLayer: 2));
            server.EntMan.InitializeAndStartEntity(body);
            worldNet = server.EntMan.GetNetEntity(world); frameNet = server.EntMan.GetNetEntity(frame); bodyNet = server.EntMan.GetNetEntity(body);
            viewer = server.EntMan.SpawnAttachedTo(null, world, RigidTransform3.Identity);
            var session = server.PlayerMan.Sessions.First();
            server.PlayerMan.SetAttachedEntity(session, viewer); server.PlayerMan.JoinGame(session);
        });
        await RunTicksSync(server, client, 40);
        SpatialCollisionView view = default!, limited = default!, entityLimited = default!, visitLimited = default!;
        SpatialClientCollisionScene previous = default!;
        EntityUid localWorld = default, localFrame = default, localBody = default;
        await client.WaitAssertion(() =>
        {
            localWorld = client.EntMan.GetEntity(worldNet); localFrame = client.EntMan.GetEntity(frameNet); localBody = client.EntMan.GetEntity(bodyNet);
            var views = client.System<SpatialCollisionViewSystem>();
            var bounds = new Box3(new Vector3(-4), new Vector3(4));
            view = views.CreateView(localWorld, bounds, new SpatialCollisionViewLimits(1000, 1000, 1000));
            limited = views.CreateView(localWorld, bounds, new SpatialCollisionViewLimits(1000, 1000, 1));
            entityLimited = views.CreateView(localWorld, bounds, new SpatialCollisionViewLimits(1000, 1, 1000));
            visitLimited = views.CreateView(localWorld, bounds, new SpatialCollisionViewLimits(1, 1000, 1000));
            Assert.That(view.TryGetScene(out _), Is.False, "Do not capture potentially predicted live poses on demand.");
        });
        await RunTicksSync(server, client, 5);
        await client.WaitAssertion(() =>
        {
            Assert.That(view.TryGetScene(out var scene), Is.True);
            previous = scene!;
            var context = new SpatialQueryContext(localWorld, localWorld, "collision-view-test", uint.MaxValue);
            var floor = scene!.FindSurface(context, new Vector3(0, 0, 3), -Vector3.UnitZ, 3.5f);
            Assert.That(floor, Is.Not.Null);
            Assert.That(floor!.Value.Surface, Is.EqualTo(new SpatialSurfaceReference(localFrame, new SurfaceId(1))));
            Assert.That(floor.Value.MaterialProfile, Is.EqualTo("metal"));
            Assert.That(floor.Value.Distance, Is.EqualTo(3).Within(1e-4));
            var onlyBody = new SpatialQueryContext(localWorld, localWorld, "body", 2);
            var hit = scene.RayCast(onlyBody, new Vector3(0, 0, 1), Vector3.UnitX, 3);
            Assert.That(hit!.Value.Entity, Is.EqualTo(localBody));
            Assert.That(hit.Value.Distance, Is.EqualTo(1.5f).Within(1e-4));
            var before = client.EntMan.GetComponent<TransformComponent>(localBody).LocalPose;
            Assert.That(scene.OverlapContacts(onlyBody, new SpatialSphereShape(0.2f),
                new RigidTransform3(new Vector3(2, 0, 1), Quaternion.Identity)), Is.Not.Empty);
            Assert.That(client.EntMan.GetComponent<TransformComponent>(localBody).LocalPose, Is.EqualTo(before));
            Assert.Throws<InvalidOperationException>(() => scene.RayCast(context, Vector3.Zero, Vector3.UnitX, 20));
            Assert.Throws<InvalidOperationException>(() => scene.ShapeCast(context, new SpatialBoxShape(Vector3.One),
                RigidTransform3.Identity, new Vector3(0, 0, 8), Vector3.UnitY));
            Assert.That(limited.TryGetScene(out _), Is.False);
            Assert.That(limited.BudgetExceeded, Is.True);
            Assert.That(entityLimited.TryGetScene(out _), Is.False);
            Assert.That(entityLimited.BudgetExceeded, Is.True);
            Assert.That(visitLimited.TryGetScene(out _), Is.False);
            Assert.That(visitLimited.BudgetExceeded, Is.True);
            view.SetBounds(new Box3(new Vector3(-1000), new Vector3(1000)));
            Assert.That(previous.IsAvailable, Is.False);
            Assert.That(view.TryGetScene(out _), Is.False);
        });
        await RunTicksSync(server, client, 5);
        await client.WaitAssertion(() =>
        {
            Assert.That(view.TryGetScene(out _), Is.False, "A view cannot expand the server's authorized coverage.");
            view.SetBounds(new Box3(new Vector3(-4), new Vector3(4)));
        });
        await RunTicksSync(server, client, 5);
        await client.WaitAssertion(() =>
        {
            Assert.That(view.TryGetScene(out var scene), Is.True);
            previous = scene!;
        });
        await server.WaitPost(() =>
        {
            var geometry = server.System<SharedGeometrySystem>();
            var slabs = new List<SlabSurface>
            {
                new(new SpatialSurfaceReference(frame, new SurfaceId(1)), layer, new Box2(-2, -2, 2, 2), 1, 0.2f, materialProfile: "raised"),
            };
            // A real multi-fragment replacement exposes the interval between the
            // entity revision and complete semantic geometry installation.
            for (var i = 0; i < 300; i++)
                slabs.Add(new SlabSurface(new SpatialSurfaceReference(frame, new SurfaceId((uint) i + 2)), layer,
                    new Box2(4 + i % 20 * 0.2f, 4 + i / 20 * 0.2f, 4.1f + i % 20 * 0.2f, 4.1f + i / 20 * 0.2f),
                    0, 0.1f, materialProfile: new string('m', 2000)));
            geometry.QueueTransaction(geometry.PrepareTransaction(new[] { new SpatialGeometryEdit(geometry.CaptureSnapshot(frame), slabs: slabs) }));
        });
        var sawUnavailable = false;
        for (var i = 0; i < 40; i++)
        {
            await RunTicksSync(server, client, 1);
            await client.WaitAssertion(() =>
            {
                if (!client.System<SpatialGeometryStreamingSystem>().IsReady(localWorld, view.Bounds))
                {
                    sawUnavailable = true;
                    Assert.That(view.TryGetScene(out _), Is.False, "Pending collision is not empty collision.");
                    Assert.That(previous.IsAvailable, Is.False);
                }
            });
        }
        await client.WaitAssertion(() =>
        {
            Assert.That(sawUnavailable, Is.True);
            Assert.That(previous.IsAvailable, Is.False);
            Assert.That(view.TryGetScene(out var scene), Is.True);
            var context = new SpatialQueryContext(localWorld, localWorld, "replacement", 1,
                expectedGeometryRevisions: new Dictionary<EntityUid, ulong> { [localFrame] = 1 });
            var hit = scene!.FindSurface(context, new Vector3(0, 0, 3), -Vector3.UnitZ, 3.5f);
            Assert.That(hit!.Value.Distance, Is.EqualTo(2).Within(1e-4));
            Assert.That(hit.Value.MaterialProfile, Is.EqualTo("raised"));
            Assert.Throws<InvalidOperationException>(() => previous.RayCast(context, Vector3.Zero, Vector3.UnitX, 1));
        });
        await server.WaitAssertion(() =>
        {
            var containers = server.System<SharedContainerSystem>();
            var container = containers.EnsureContainer<ContainerSlot>(viewer, "collision-test");
            Assert.That(containers.Insert(body, container), Is.True);
        });
        await RunTicksSync(server, client, 10);
        await client.WaitAssertion(() =>
        {
            Assert.That(view.TryGetScene(out var scene), Is.True);
            var context = new SpatialQueryContext(localWorld, localWorld, "contained-body", 2);
            Assert.That(scene!.RayCast(context, Vector3.Zero, Vector3.UnitX, 3), Is.Null,
                "A contained body is dormant even though its entity is still known.");
            Assert.That(scene.Overlap(context, new SpatialSphereShape(0.2f), RigidTransform3.Identity), Is.Empty);
        });
        await DisconnectClient(server, client);
        await client.WaitAssertion(() =>
        {
            Assert.That(view.IsDisposed, Is.True);
            Assert.That(view.TryGetScene(out _), Is.False);
            Assert.That(limited.IsDisposed, Is.True);
            Assert.That(entityLimited.IsDisposed, Is.True);
            Assert.That(visitLimited.IsDisposed, Is.True);
        });
    }
}
