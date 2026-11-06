using System.Numerics;
using NUnit.Framework;
using Robust.Shared.EntitySerialization.Systems;
using Robust.Shared.GameObjects;
using Robust.Shared.Map.Components;
using Robust.Shared.Network;
using Robust.Shared.Player;
using Robust.Shared.Spatial;
using Robust.Shared.Spatial.Physics;
using Robust.Shared.Utility;
using Robust.UnitTesting.Shared.GameObjects.Systems;

namespace Robust.UnitTesting.Shared.EntitySerialization;

[TestFixture]
public sealed class SpatialMotorPersistenceTest : RobustIntegrationTest
{
    [Test]
    public async Task SavedMotorRestoresControllerAndResizedBodyWithoutAnotherPose()
    {
        using var server = StartServer(new ServerIntegrationOptions { Pool = false });
        await server.WaitIdleAsync();
        await server.WaitAssertion(() =>
        {
            var entities = server.EntMan;
            var physics = server.System<SharedSpatialPhysicsSystem>();
            var world = server.System<SharedSpatialSystem>().CreateWorld();
            physics.EnablePhysics(world, new SpatialPhysicsSettings(Vector3.Zero, 1, 8));
            var intent = new SpatialMotorIntent(new Vector3(1, 2, 3), SpatialMotorMode.Free);
            var motor = SpatialMotorTests.SpawnMotor(entities, world, Vector3.UnitZ * 5, intent);
            var settings = SpatialMotorSettings.Default with { StepHeight = 0.42f };
            physics.SetMotorSettings(motor, settings);
            Assert.That(physics.TryResizeMotor(motor, 0.2f, 0.6f, 0), Is.True);
            var path = new ResPath("/spatial-motor.yml");
            var loader = server.System<MapLoaderSystem>();
            Assert.That(loader.TrySaveMap(entities.GetComponent<MapComponent>(world).MapId, path), Is.True);
            entities.DeleteEntity(world);
            Assert.That(loader.TryLoadMap(path, out var restored, out _), Is.True);
            using var children = entities.GetComponent<TransformComponent>(restored!.Value.Owner).ChildEnumerator;
            Assert.That(children.MoveNext(out var uid), Is.True);
            Assert.That(children.MoveNext(out _), Is.False);
            var loaded = entities.GetComponent<SpatialMotorComponent>(uid);
            var body = entities.GetComponent<SpatialBodyComponent>(uid);
            Assert.That(loaded.Intent, Is.EqualTo(intent));
            Assert.That(loaded.Settings, Is.EqualTo(settings));
            Assert.That(loaded.Support, Is.Null);
            Assert.That(body.Definition.Owner, Is.EqualTo(SpatialMotionOwner.Motor));
            Assert.That(body.DefinitionRevision, Is.EqualTo(1));
            Assert.That(((SpatialCapsuleShape) body.Definition.Colliders[0].Shape).Length, Is.EqualTo(0.6f));
            Assert.That(body.Definition.Colliders[0].Offset.Z, Is.EqualTo(-0.3f).Within(1e-6));
            Assert.That(physics.GetBodyCount(restored.Value), Is.EqualTo(1));
            physics.Update(0.1f);
            Assert.That(server.System<SharedTransformSystem>().GetWorldPose(uid).Position.Z, Is.GreaterThan(5));
            Assert.That(body.AngularVelocity, Is.EqualTo(Vector3.Zero));
        });
    }

    [Test]
    public async Task ConnectedClientReceivesMotorIntentAndCannotAuthoritativelyResizeIt()
    {
        using var server = StartServer(new ServerIntegrationOptions { Pool = false });
        using var client = StartClient(new ClientIntegrationOptions { Pool = false });
        await Task.WhenAll(server.WaitIdleAsync(), client.WaitIdleAsync());
        EntityUid uid = default;
        NetEntity motorNet = default;
        NetEntity floorNet = default;
        var intent = new SpatialMotorIntent(Vector3.UnitX, SpatialMotorMode.Walking);
        await server.WaitAssertion(() =>
        {
            var world = server.System<SharedSpatialSystem>().CreateWorld();
            server.System<SharedSpatialPhysicsSystem>().EnablePhysics(world);
            floorNet = server.EntMan.GetNetEntity(SpatialMotorTests.Floor(server.EntMan, world));
            var motor = SpatialMotorTests.SpawnMotor(server.EntMan, world, Vector3.UnitZ * 0.8f, intent);
            uid = motor;
            motorNet = server.EntMan.GetNetEntity(uid);
            Assert.That(server.System<SharedSpatialPhysicsSystem>().TryResizeMotor(motor, 0.25f, 0.8f, 0), Is.True);
        });
        client.SetConnectTarget(server);
        await client.WaitPost(() => client.Resolve<IClientNetManager>().ClientConnect(null!, 0, null!));
        await RunTicksSync(server, client, 10);
        await server.WaitPost(() =>
        {
            var players = server.Resolve<ISharedPlayerManager>();
            var session = players.Sessions.First();
            server.PlayerMan.SetAttachedEntity(session, uid);
            players.JoinGame(session);
        });
        await RunTicksSync(server, client, 30);
        await client.WaitAssertion(() =>
        {
            var local = client.EntMan.GetEntity(motorNet);
            var motor = client.EntMan.GetComponent<SpatialMotorComponent>(local);
            var body = client.EntMan.GetComponent<SpatialBodyComponent>(local);
            var physics = client.System<SharedSpatialPhysicsSystem>();
            Assert.That(motor.Intent, Is.EqualTo(intent));
            Assert.That(motor.Settings, Is.EqualTo(SpatialMotorSettings.Default));
            Assert.That(body.Definition.Owner, Is.EqualTo(SpatialMotionOwner.Motor));
            Assert.That(body.DefinitionRevision, Is.EqualTo(1));
            Assert.That(body.LinearVelocity.X, Is.EqualTo(1).Within(0.001));
            Assert.That(motor.Support, Is.Not.Null);
            Assert.That(motor.Support!.Value.Source.Entity, Is.EqualTo(client.EntMan.GetEntity(floorNet)));
            Assert.That(motor.Support.Value.Source.Surface,
                Is.EqualTo(new SpatialSurfaceReference(client.EntMan.GetEntity(floorNet), new SurfaceId(1))));
            var pose = client.System<SharedTransformSystem>().GetWorldPose(local);
            Assert.That(pose.Position.X, Is.GreaterThan(0));
            Assert.Throws<InvalidOperationException>(() => physics.SetMotorIntent((local, motor), default));
            Assert.Throws<InvalidOperationException>(() => physics.TryResizeMotor((local, motor), 0.3f, 1, 1));
            physics.Update(1);
            Assert.That(client.System<SharedTransformSystem>().GetWorldPose(local), Is.EqualTo(pose));
        });
    }
}
