using NUnit.Framework;
using Robust.Shared.Configuration;
using Robust.Shared.EntitySerialization;
using Robust.Shared.EntitySerialization.Systems;
using Robust.Shared.GameObjects;
using Robust.Shared.Log;
using Robust.Shared.Map.Components;
using Robust.Shared.Maths;
using Robust.Shared.Network;
using Robust.Shared.Player;
using Robust.Shared.Serialization;
using Robust.Shared.Serialization.Manager.Attributes;
using Robust.Shared.Spatial;
using Robust.Shared.Utility;
using Robust.UnitTesting.Shared.GameObjects.Systems;

namespace Robust.UnitTesting.Shared.EntitySerialization;

[RegisterComponent]
internal sealed partial class SpatialLayerReferenceSaveTestComponent : Component
{
    [DataField] public SpatialLayerReference Layer;
}

[TestFixture]
public sealed class SpatialWorldPersistenceTest : RobustIntegrationTest
{
    [Test]
    public async Task ConnectedClientReceivesWorldLayersAndReleasesDeletedIdentity()
    {
        using var server = StartServer(new ServerIntegrationOptions { Pool = false });
        using var client = StartClient(new ClientIntegrationOptions { Pool = false });
        await Task.WhenAll(server.WaitIdleAsync(), client.WaitIdleAsync());
        EntityUid world = default;
        EntityUid observer = default;
        NetEntity worldNet = default;
        NetEntity observerNet = default;
        NetSpatialLayerReference layerNet = default;
        PersistentWorldId worldId = default;
        var definition = new LayerDefinition(2.75f, "Mezzanine", 0.5f);
        await server.WaitAssertion(() =>
        {
            var spatial = server.System<SharedSpatialSystem>();
            var created = spatial.CreateWorld();
            world = created.Owner;
            worldId = created.Comp.PersistentId;
            worldNet = server.EntMan.GetNetEntity(world);
            var layer = new LayerId(80);
            var frame = SpatialWorldTests.CreateFrame(server.EntMan, world,
                new Dictionary<LayerId, LayerDefinition> { [layer] = definition });
            observer = server.EntMan.SpawnAttachedTo(null, frame,
                new RigidTransform3(new System.Numerics.Vector3(1, 2, 4), System.Numerics.Quaternion.Identity));
            observerNet = server.EntMan.GetNetEntity(observer);
            layerNet = spatial.ToNetwork(new SpatialLayerReference(frame, layer));
        });
        client.SetConnectTarget(server);
        await client.WaitPost(() => client.Resolve<IClientNetManager>().ClientConnect(null!, 0, null!));
        await RunTicksSync(server, client, 10);
        await server.WaitPost(() =>
        {
            var players = server.Resolve<ISharedPlayerManager>();
            var session = players.Sessions.First();
            server.PlayerMan.SetAttachedEntity(session, observer);
            players.JoinGame(session);
        });
        await RunTicksSync(server, client, 20);
        await client.WaitAssertion(() =>
        {
            var entities = client.EntMan;
            var spatial = client.System<SharedSpatialSystem>();
            var entityCount = entities.EntityCount;
            Assert.Throws<InvalidOperationException>(() => spatial.CreateWorld());
            Assert.That(entities.EntityCount, Is.EqualTo(entityCount));
            var localWorld = entities.GetEntity(worldNet);
            Assert.That(entities.GetComponent<SpatialWorldComponent>(localWorld).PersistentId, Is.EqualTo(worldId));
            Assert.That(spatial.TryResolveWorld(worldId, out var registered), Is.True);
            Assert.That(registered, Is.EqualTo(localWorld));
            var reference = spatial.FromNetwork(layerNet);
            Assert.That(spatial.GetLayer(reference), Is.EqualTo(definition));
            var relative = spatial.GetPoseRelativeToLayer(entities.GetEntity(observerNet), reference);
            Assert.That(relative.Position, Is.EqualTo(new System.Numerics.Vector3(1, 2, 1.25f)));
        });
        await server.WaitPost(() => server.EntMan.DeleteEntity(world));
        await RunTicksSync(server, client, 20);
        await client.WaitAssertion(() =>
            Assert.That(client.System<SharedSpatialSystem>().TryResolveWorld(worldId, out _), Is.False));
    }

    [Test]
    public async Task SaveRestoreAndExplicitClonePreserveScopedReferencesWithoutIdentityCollision()
    {
        using var server = StartServer(new ServerIntegrationOptions { Pool = false });
        await server.WaitIdleAsync();
        await server.WaitAssertion(() =>
        {
            var entities = server.EntMan;
            var spatial = server.System<SharedSpatialSystem>();
            var loader = server.System<MapLoaderSystem>();
            var world = spatial.CreateWorld();
            var originalId = world.Comp.PersistentId;
            var layer = new LayerId(51);
            var definition = new LayerDefinition(-3.25f, "Lower deck", 0.5f);
            var frame = SpatialWorldTests.CreateFrame(entities, world,
                new Dictionary<LayerId, LayerDefinition> { [layer] = definition });
            var child = entities.SpawnAttachedTo(null, frame, RigidTransform3.Identity);
            entities.EnsureComponent<SpatialLayerReferenceSaveTestComponent>(child).Layer = new SpatialLayerReference(frame, layer);
            var path = new ResPath("/spatial-world.yml");
            Assert.That(loader.TrySaveMap(entities.GetComponent<MapComponent>(world).MapId, path), Is.True);
            entities.DeleteEntity(world);
            Assert.That(spatial.TryResolveWorld(originalId, out _), Is.False);

            Assert.That(loader.TryLoadMap(path, out var restored, out _), Is.True);
            Assert.That(entities.GetComponent<SpatialWorldComponent>(restored!.Value).PersistentId, Is.EqualTo(originalId));
            Assert.That(spatial.TryResolveWorld(originalId, out var resolved), Is.True);
            Assert.That(resolved, Is.EqualTo(restored.Value.Owner));

            // The same persistent world cannot silently acquire a second live identity.
            var count = entities.EntityCount;
            var cfg = server.Resolve<IConfigurationManager>();
            var failure = cfg.GetCVar(RTCVars.FailureLogLevel);
            try
            {
                cfg.SetCVar(RTCVars.FailureLogLevel, LogLevel.Fatal);
                Assert.Throws<InvalidOperationException>(() => loader.TryLoadMap(path, out _, out _));
            }
            finally
            {
                cfg.SetCVar(RTCVars.FailureLogLevel, failure);
            }
            Assert.That(entities.EntityCount, Is.EqualTo(count));
            Assert.That(spatial.TryResolveWorld(originalId, out resolved), Is.True);
            Assert.That(resolved, Is.EqualTo(restored.Value.Owner));

            Assert.That(loader.TryLoadMap(path, out var clone, out _,
                new DeserializationOptions { RegenerateSpatialWorldIds = true }), Is.True);
            Assert.That(entities.GetComponent<SpatialWorldComponent>(clone!.Value).PersistentId, Is.Not.EqualTo(originalId));
            var references = new HashSet<SpatialLayerReference>();
            foreach (var map in new[] { restored.Value.Owner, clone.Value.Owner })
            {
                var newFrame = OnlyChild(entities, map);
                var newChild = OnlyChild(entities, newFrame);
                var reference = entities.GetComponent<SpatialLayerReferenceSaveTestComponent>(newChild).Layer;
                Assert.That(reference.OwnerFrame, Is.EqualTo(newFrame));
                Assert.That(reference.Layer, Is.EqualTo(layer));
                Assert.That(newFrame, Is.Not.EqualTo(frame));
                Assert.That(spatial.GetLayer(reference), Is.EqualTo(definition));
                references.Add(reference);
            }
            Assert.That(references, Has.Count.EqualTo(2));

            var serializer = server.Resolve<IRobustSerializer>();
            using var stream = new MemoryStream();
            var worldState = new SpatialWorldComponentState(originalId, SpatialWorldComponent.CurrentDefinitionSchema);
            serializer.Serialize(stream, worldState);
            stream.Position = 0;
            Assert.That(serializer.Deserialize<SpatialWorldComponentState>(stream).PersistentId, Is.EqualTo(originalId));
            stream.SetLength(0);
            stream.Position = 0;
            var frameState = new SpatialFrameComponentState(new Dictionary<LayerId, LayerDefinition> { [layer] = definition });
            serializer.Serialize(stream, frameState);
            stream.Position = 0;
            Assert.That(serializer.Deserialize<SpatialFrameComponentState>(stream).Layers[layer], Is.EqualTo(definition));
        });
    }

    private static EntityUid OnlyChild(IEntityManager entities, EntityUid parent)
    {
        using var children = entities.GetComponent<TransformComponent>(parent).ChildEnumerator;
        Assert.That(children.MoveNext(out var child), Is.True);
        Assert.That(children.MoveNext(out _), Is.False);
        return child;
    }
}
