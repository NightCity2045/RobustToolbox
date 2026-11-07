using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Threading.Tasks;
using NUnit.Framework;
using Robust.Server.GameObjects;
using Robust.Shared;
using Robust.Shared.GameObjects;
using Robust.Shared.Maths;
using Robust.Shared.Network;
using Robust.Shared.Spatial;
using Robust.Shared.Spatial.Networking;
using Robust.Shared.Spatial.Physics;
using ClientStreaming = Robust.Client.Spatial.SpatialGeometryStreamingSystem;

namespace Robust.UnitTesting.Server.GameStates;

[TestFixture]
public sealed class SpatialGeometryStreamingTest : RobustIntegrationTest
{
    [TestCase(true)]
    [TestCase(false)]
    public async Task LargeBaselineProgressResetAndMultiFrameTransaction(bool pvs)
    {
        await using var pair = await StartConnectedPair();
        var (client, server) = pair;
        await RunTicksSync(server, client, 10);
        EntityUid world = default, first = default, second = default, viewer = default;
        NetEntity worldNet = default, firstNet = default, secondNet = default;
        var layer = new LayerId(1);
        await server.WaitPost(() =>
        {
            server.CfgMan.SetCVar(CVars.NetPVS, pvs);
            server.CfgMan.SetCVar(CVars.NetSpatialGeometryBytes, 65536);
            world = server.System<SharedSpatialSystem>().CreateWorld();
            first = CreateFrame(server.EntMan, world, 200);
            second = CreateFrame(server.EntMan, world, 1);
            worldNet = server.EntMan.GetNetEntity(world);
            firstNet = server.EntMan.GetNetEntity(first);
            secondNet = server.EntMan.GetNetEntity(second);
            viewer = server.EntMan.SpawnAttachedTo(null, world, RigidTransform3.Identity);
            var session = server.PlayerMan.Sessions.First();
            server.PlayerMan.SetAttachedEntity(session, viewer);
            server.PlayerMan.JoinGame(session);
        });
        // The first frame exceeds the 256 KiB outstanding fragment window. This
        // cannot complete unless the real message path returns progress ACKs.
        await RunTicksSync(server, client, 70);
        ulong sequence = 0;
        await client.WaitAssertion(() =>
        {
            var streaming = client.System<ClientStreaming>();
            Assert.That(streaming.TryGetSnapshot(client.EntMan.GetEntity(firstNet), out var snapshot), Is.True);
            Assert.That(snapshot!.Slabs.Count, Is.EqualTo(200));
            Assert.That(snapshot.OwnerFrame, Is.EqualTo(client.EntMan.GetEntity(firstNet)));
            Assert.That(streaming.TryGetSnapshot(client.EntMan.GetEntity(secondNet), out _), Is.True);
            Assert.That(streaming.IsReady(client.EntMan.GetEntity(worldNet), new Box3(new Vector3(-1), new Vector3(1))), Is.True);
            Assert.That(streaming.IsReady(client.EntMan.GetEntity(worldNet), new Box3(new Vector3(999), new Vector3(1000))), Is.False);
            sequence = streaming.Sequence;
            client.ResolveDependency<IClientNetManager>().ClientSendMessage(new MsgSpatialGeometryRequestBaseline { LastSequence = sequence });
        });
        await RunTicksSync(server, client, 70);
        await client.WaitAssertion(() =>
        {
            var streaming = client.System<ClientStreaming>();
            Assert.That(streaming.Sequence, Is.GreaterThan(sequence), "A full reset must not stall on cached payload ACKs.");
            Assert.That(streaming.IsReady(client.EntMan.GetEntity(worldNet), new Box3(new Vector3(-1), new Vector3(1))), Is.True);
        });
        await server.WaitPost(() =>
        {
            var geometry = server.System<SharedGeometrySystem>();
            var transaction = geometry.PrepareTransaction(new[] { first, second }.Select(frame =>
                new SpatialGeometryEdit(geometry.CaptureSnapshot(frame), layers:
                    new Dictionary<LayerId, LayerDefinition> { [layer] = new(2, "Raised floor") })));
            geometry.QueueTransaction(transaction);
        });
        await RunTicksSync(server, client, 70);
        await client.WaitAssertion(() =>
        {
            var streaming = client.System<ClientStreaming>();
            foreach (var frame in new[] { firstNet, secondNet })
            {
                Assert.That(streaming.TryGetSnapshot(client.EntMan.GetEntity(frame), out var snapshot), Is.True);
                Assert.That(snapshot!.Revision, Is.EqualTo(1));
                Assert.That(snapshot.Layers[layer].LocalElevation, Is.EqualTo(2));
            }
            Assert.That(streaming.IsReady(client.EntMan.GetEntity(worldNet), new Box3(new Vector3(-1), new Vector3(1))), Is.True);
        });
        ClientStreaming disconnectedReplica = default!;
        await client.WaitPost(() => disconnectedReplica = client.System<ClientStreaming>());
        await DisconnectClient(server, client);
        await client.WaitAssertion(() => Assert.That(disconnectedReplica.Sequence, Is.Zero));
        await server.WaitPost(() =>
        {
            // Geometry changes while this client has no connection. The next
            // session must establish a fresh baseline, not resurrect its cache.
            var geometry = server.System<SharedGeometrySystem>();
            geometry.QueueTransaction(geometry.PrepareTransaction(new[]
            {
                new SpatialGeometryEdit(geometry.CaptureSnapshot(first), slabs: Array.Empty<SlabSurface>())
            }));
        });
        await ConnectClient(server, client);
        await RunTicksSync(server, client, 10);
        await server.WaitPost(() =>
        {
            var session = server.PlayerMan.Sessions.First();
            server.PlayerMan.SetAttachedEntity(session, viewer);
            server.PlayerMan.JoinGame(session);
        });
        await RunTicksSync(server, client, 40);
        await client.WaitAssertion(() =>
        {
            var streaming = client.System<ClientStreaming>();
            Assert.That(streaming.TryGetSnapshot(client.EntMan.GetEntity(firstNet), out _), Is.False,
                "Removed geometry must not survive the disconnected session.");
            Assert.That(streaming.TryGetSnapshot(client.EntMan.GetEntity(secondNet), out var snapshot), Is.True);
            Assert.That(snapshot!.Revision, Is.EqualTo(1));
            Assert.That(streaming.IsReady(client.EntMan.GetEntity(worldNet), new Box3(new Vector3(-1), new Vector3(1))), Is.True);
        });
    }

    [TestCase(true)]
    [TestCase(false)]
    public async Task OversizedFrameTransfersEveryPartBeforePublishingCollision(bool pvs)
    {
        await using var pair = await StartConnectedPair();
        var (client, server) = pair;
        await RunTicksSync(server, client, 10);
        NetEntity frameNet = default, worldNet = default;
        await server.WaitPost(() =>
        {
            server.CfgMan.SetCVar(CVars.NetPVS, pvs);
            server.CfgMan.SetCVar(CVars.NetSpatialGeometryBytes, 1024 * 1024);
            var world = server.System<SharedSpatialSystem>().CreateWorld();
            var frame = CreateFrame(server.EntMan, world, 4500, compact: true);
            frameNet = server.EntMan.GetNetEntity(frame);
            worldNet = server.EntMan.GetNetEntity(world);
            var selected = server.System<SharedGeometrySystem>().CaptureSnapshot(frame);
            Assert.That(SpatialGeometryWireCodec.EncodeParts(selected, uid => server.EntMan.GetNetEntity(uid)).Count, Is.GreaterThan(1));
            var viewer = server.EntMan.SpawnAttachedTo(null, world, RigidTransform3.Identity);
            var session = server.PlayerMan.Sessions.First();
            server.PlayerMan.SetAttachedEntity(session, viewer);
            server.PlayerMan.JoinGame(session);
        });
        var waited = false;
        for (var tick = 0; tick < 100; tick++)
        {
            await RunTicksSync(server, client, 1);
            await client.WaitAssertion(() =>
            {
                var streaming = client.System<ClientStreaming>();
                if (!client.EntMan.TryGetEntity(frameNet, out var frame) || !streaming.TryGetSnapshot(frame.Value, out var snapshot))
                { waited = true; return; }
                Assert.That(snapshot.Slabs.Count, Is.EqualTo(4500), "A frame must never expose only its first part.");
                Assert.That(streaming.IsReady(client.EntMan.GetEntity(worldNet), new Box3(new Vector3(-1), new Vector3(1))), Is.True);
            });
        }
        ulong sequence = 0;
        await client.WaitAssertion(() =>
        {
            var streaming = client.System<ClientStreaming>();
            Assert.That(waited, Is.True);
            Assert.That(streaming.TryGetSnapshot(client.EntMan.GetEntity(frameNet), out var snapshot), Is.True);
            Assert.That(snapshot!.Slabs.Count, Is.EqualTo(4500));
            sequence = streaming.Sequence;
            client.ResolveDependency<IClientNetManager>().ClientSendMessage(new MsgSpatialGeometryRequestBaseline { LastSequence = sequence });
        });
        await RunTicksSync(server, client, 100);
        await client.WaitAssertion(() =>
        {
            var streaming = client.System<ClientStreaming>();
            Assert.That(streaming.Sequence, Is.GreaterThan(sequence));
            Assert.That(streaming.TryGetSnapshot(client.EntMan.GetEntity(frameNet), out var snapshot), Is.True);
            Assert.That(snapshot!.Slabs.Count, Is.EqualTo(4500));
            Assert.That(streaming.IsReady(client.EntMan.GetEntity(worldNet), new Box3(new Vector3(-1), new Vector3(1))), Is.True);
        });
    }

    [Test]
    public async Task DeniedSurfaceDoesNotLeakAndDoesNotCertifyEmptyCollision()
    {
        await using var pair = await StartConnectedPair();
        var (client, server) = pair;
        await RunTicksSync(server, client, 10);
        EntityUid hidden = default;
        NetEntity worldNet = default, publicNet = default, hiddenFrameNet = default;
        await server.WaitPost(() =>
        {
            server.CfgMan.SetCVar(CVars.NetPVS, true);
            var world = server.System<SharedSpatialSystem>().CreateWorld();
            worldNet = server.EntMan.GetNetEntity(world);
            publicNet = server.EntMan.GetNetEntity(CreateFrame(server.EntMan, world, 1));
            hidden = server.EntMan.SpawnAttachedTo(null, world, RigidTransform3.Identity);
            var visibility = server.EntMan.AddComponent<VisibilityComponent>(hidden);
            server.System<VisibilitySystem>().AddLayer((hidden, visibility), 2);
            hiddenFrameNet = server.EntMan.GetNetEntity(CreateFrame(server.EntMan, world, 1, hidden));
            var viewer = server.EntMan.SpawnAttachedTo(null, world, RigidTransform3.Identity);
            var session = server.PlayerMan.Sessions.First();
            server.PlayerMan.SetAttachedEntity(session, viewer);
            server.PlayerMan.JoinGame(session);
        });
        await RunTicksSync(server, client, 30);
        await client.WaitAssertion(() =>
        {
            var streaming = client.System<ClientStreaming>();
            Assert.That(streaming.TryGetSnapshot(client.EntMan.GetEntity(publicNet), out _), Is.True);
            Assert.That(streaming.TryGetSnapshot(client.EntMan.GetEntity(hiddenFrameNet), out _), Is.False);
            Assert.That(streaming.IsReady(client.EntMan.GetEntity(worldNet), new Box3(new Vector3(-1), new Vector3(1))), Is.False);
        });
        await server.WaitPost(() => server.System<VisibilitySystem>().RemoveLayer(hidden, 2));
        await RunTicksSync(server, client, 30);
        await client.WaitAssertion(() =>
        {
            var streaming = client.System<ClientStreaming>();
            Assert.That(streaming.TryGetSnapshot(client.EntMan.GetEntity(hiddenFrameNet), out var snapshot), Is.True);
            Assert.That(snapshot!.Slabs.Values.Single().Entity, Is.Not.Null);
            Assert.That(streaming.IsReady(client.EntMan.GetEntity(worldNet), new Box3(new Vector3(-1), new Vector3(1))), Is.True);
        });
    }

    [Test]
    public async Task WorldTransferInvalidatesOldGeometryBeforeInstallingNewCoverage()
    {
        await using var pair = await StartConnectedPair();
        var (client, server) = pair;
        await RunTicksSync(server, client, 10);
        EntityUid source = default, frame = default, viewer = default;
        NetEntity frameNet = default, destinationNet = default;
        await server.WaitPost(() =>
        {
            source = server.System<SharedSpatialSystem>().CreateWorld();
            server.System<SharedSpatialPhysicsSystem>().EnablePhysics(source);
            frame = CreateFrame(server.EntMan, source, 1);
            frameNet = server.EntMan.GetNetEntity(frame);
            viewer = server.EntMan.SpawnAttachedTo(null, source, RigidTransform3.Identity);
            var session = server.PlayerMan.Sessions.First();
            server.PlayerMan.SetAttachedEntity(session, viewer);
            server.PlayerMan.JoinGame(session);
        });
        await RunTicksSync(server, client, 40);
        var observed = false;
        await client.WaitAssertion(() =>
        {
            var streaming = client.System<ClientStreaming>();
            var localFrame = client.EntMan.GetEntity(frameNet);
            Assert.That(streaming.TryGetSnapshot(localFrame, out _), Is.True);
            client.System<SharedTransformSystem>().OnGlobalMoveEvent += (ref MoveEvent ev) =>
            {
                if (ev.Sender != localFrame || !ev.ParentChanged) return;
                observed = true;
                Assert.That(streaming.TryGetSnapshot(localFrame, out _), Is.False,
                    "The old world cache must be invalidated during the parent update, even with the same geometry stamp.");
                if (ev.Component.MapUid is { } world)
                    Assert.That(streaming.IsReady(world, new Box3(new Vector3(-1), new Vector3(1))), Is.False);
            };
        });
        await server.WaitPost(() =>
        {
            var destination = server.System<SharedSpatialSystem>().CreateWorld();
            var physics = server.System<SharedSpatialPhysicsSystem>();
            physics.EnablePhysics(destination);
            destinationNet = server.EntMan.GetNetEntity(destination);
            physics.TransferGraph(new[] { frame, viewer }, destination, RigidTransform3.Identity);
        });
        await RunTicksSync(server, client, 40);
        await client.WaitAssertion(() =>
        {
            Assert.That(observed, Is.True);
            var streaming = client.System<ClientStreaming>();
            Assert.That(streaming.TryGetSnapshot(client.EntMan.GetEntity(frameNet), out var snapshot), Is.True);
            Assert.That(snapshot!.Revision, Is.Zero);
            Assert.That(streaming.IsReady(client.EntMan.GetEntity(destinationNet), new Box3(new Vector3(-1), new Vector3(1))), Is.True);
        });
    }

    private static EntityUid CreateFrame(IEntityManager entities, EntityUid world, int slabs, EntityUid? source = null, bool compact = false)
    {
        var uid = entities.CreateEntityUninitialized(null, world, RigidTransform3.Identity);
        var layer = new LayerId(1);
        var frame = entities.AddComponent<SpatialFrameComponent>(uid);
        entities.System<SharedSpatialSystem>().ConfigureInitialLayers((uid, frame),
            new Dictionary<LayerId, LayerDefinition> { [layer] = new(0, "Floor") });
        var geometry = entities.AddComponent<SpatialGeometryComponent>(uid);
        var columns = compact ? 100 : 20;
        var spacing = compact ? 0.1f : 0.5f;
        entities.System<SharedGeometrySystem>().ConfigureInitialGeometry((uid, geometry), Array.Empty<BoundarySegment>(),
            Enumerable.Range(0, slabs).Select(i => new SlabSurface(new SpatialSurfaceReference(uid, new SurfaceId((uint) i)),
                layer, new Box2(i % columns * spacing, i / columns * spacing, (i % columns + 0.8f) * spacing, (i / columns + 0.8f) * spacing),
                0, 0.2f, materialProfile: slabs > 1 ? new string('m', 2000) : "metal", entity: source)));
        entities.InitializeAndStartEntity(uid);
        return uid;
    }
}
