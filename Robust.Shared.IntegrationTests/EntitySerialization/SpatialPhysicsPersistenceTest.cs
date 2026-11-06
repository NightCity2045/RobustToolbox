using System.Numerics;
using NUnit.Framework;
using Robust.Shared.ContentPack;
using Robust.Shared.EntitySerialization;
using Robust.Shared.EntitySerialization.Systems;
using Robust.Shared.GameObjects;
using Robust.Shared.Map.Components;
using Robust.Shared.Maths;
using Robust.Shared.Network;
using Robust.Shared.Player;
using Robust.Shared.Serialization;
using Robust.Shared.Spatial;
using Robust.Shared.Spatial.Physics;
using Robust.Shared.Utility;
using Robust.UnitTesting.Shared.GameObjects.Systems;

namespace Robust.UnitTesting.Shared.EntitySerialization;

[TestFixture]
public sealed class SpatialPhysicsPersistenceTest : RobustIntegrationTest
{
    [Test]
    public async Task ConstraintSaveRestoreAndCloneRemapBothParticipants()
    {
        using var server = StartServer(new ServerIntegrationOptions { Pool = false });
        await server.WaitIdleAsync();
        await server.WaitAssertion(() =>
        {
            var entities = server.EntMan;
            var physics = server.System<SharedSpatialPhysicsSystem>();
            var world = server.System<SharedSpatialSystem>().CreateWorld();
            physics.EnablePhysics(world, new SpatialPhysicsSettings(Vector3.Zero, 1, 8));
            var a = SpatialPhysicsTests.SpawnBody(entities, world, Vector3.Zero);
            var b = SpatialPhysicsTests.SpawnBody(entities, world, Vector3.UnitX * 2);
            SpatialConstraintTests.SpawnConstraint(entities, world, new SpatialConstraintDefinition(a, b,
                SpatialConstraintKind.Fixed, relativePose: new RigidTransform3(Vector3.UnitX * 2, Quaternion.Identity)));
            physics.Update(1f / 30);
            var path = new ResPath("/spatial-constraints.yml");
            var loader = server.System<MapLoaderSystem>();
            Assert.That(loader.TrySaveMap(entities.GetComponent<MapComponent>(world).MapId, path), Is.True);
            entities.DeleteEntity(world);
            Assert.That(loader.TryLoadMap(path, out var restored, out _), Is.True);
            Assert.That(loader.TryLoadMap(path, out var cloned, out _,
                new DeserializationOptions { RegenerateSpatialWorldIds = true }), Is.True);
            var participants = new HashSet<EntityUid>();
            physics.Update(1f / 30);
            foreach (var map in new[] { restored!.Value.Owner, cloned!.Value.Owner })
            {
                using var children = entities.GetComponent<TransformComponent>(map).ChildEnumerator;
                var count = 0;
                while (children.MoveNext(out var child))
                {
                    if (!entities.TryGetComponent<SpatialConstraintComponent>(child, out var constraint))
                        continue;
                    count++;
                    var d = constraint.Definition;
                    Assert.That(physics.IsConstraintActive(child), Is.True);
                    Assert.That(d.BodyA, Is.Not.EqualTo(a));
                    Assert.That(d.BodyB, Is.Not.EqualTo(b));
                    Assert.That(entities.GetComponent<TransformComponent>(d.BodyA).MapUid, Is.EqualTo(map));
                    Assert.That(entities.GetComponent<TransformComponent>(d.BodyB).MapUid, Is.EqualTo(map));
                    Assert.That(participants.Add(d.BodyA), Is.True);
                    Assert.That(participants.Add(d.BodyB), Is.True);
                    Assert.That(d.RelativePose.Position, Is.EqualTo(Vector3.UnitX * 2));
                }
                Assert.That(count, Is.EqualTo(1));
            }
            Assert.That(participants, Has.Count.EqualTo(4));
        });
    }

    [Test]
    public async Task SavedBodiesRestoreDefinitionsVelocitiesAndIndependentBackendSources()
    {
        using var server = StartServer(new ServerIntegrationOptions { Pool = false });
        await server.WaitIdleAsync();
        await server.WaitAssertion(() =>
        {
            var entities = server.EntMan;
            var physics = server.System<SharedSpatialPhysicsSystem>();
            var world = server.System<SharedSpatialSystem>().CreateWorld();
            var settings = new SpatialPhysicsSettings(new Vector3(0, 0, -2.75f), 2, 8);
            physics.EnablePhysics(world, settings);
            var definition = new SpatialBodyDefinition(SpatialMotionOwner.DynamicBody, new[]
            {
                new SpatialColliderDefinition(new SpatialBoxShape(new Vector3(0.4f, 0.5f, 0.6f)), RigidTransform3.Identity),
                new SpatialColliderDefinition(new SpatialSphereShape(0.2f), new RigidTransform3(Vector3.UnitX, Quaternion.Identity), 2),
                new SpatialColliderDefinition(new SpatialCapsuleShape(0.25f, 1.2f), new RigidTransform3(-Vector3.UnitX,
                    Quaternion.CreateFromAxisAngle(Vector3.UnitZ, 0.5f)))
            }, mass: 3, collisionLayer: 4, collisionMask: 7);
            var pose = new RigidTransform3(new Vector3(4, 5, 8), Quaternion.CreateFromAxisAngle(Vector3.UnitX, 0.3f));
            var velocity = new Vector3(1, 2, 3);
            var angular = new Vector3(0.1f, 0.2f, 0.3f);
            var body = entities.CreateEntityUninitialized(null, world, pose);
            var component = entities.AddComponent<SpatialBodyComponent>(body);
            physics.ConfigureInitialBody((body, component), definition, velocity, angular);
            entities.InitializeAndStartEntity(body, entities.GetComponent<MapComponent>(world).MapId);
            physics.ReconfigureBody((body, component), definition, 0, velocity, angular);
            var path = new ResPath("/spatial-physical-world.yml");
            var loader = server.System<MapLoaderSystem>();
            Assert.That(loader.TrySaveMap(entities.GetComponent<MapComponent>(world).MapId, path), Is.True);
            using (var reader = server.Resolve<IResourceManager>().UserData.OpenText(path))
            {
                var saved = reader.ReadToEnd();
                Assert.That(saved, Does.Contain("SpatialPhysicsWorld"));
                Assert.That(saved, Does.Contain("SpatialCapsuleShape"));
                Assert.That(saved, Does.Not.Contain("bodyHandle:"));
                Assert.That(saved, Does.Not.Contain("shapeIndex:"));
            }
            entities.DeleteEntity(world);
            Assert.That(loader.TryLoadMap(path, out var restored, out _), Is.True);
            Assert.That(loader.TryLoadMap(path, out var clone, out _,
                new DeserializationOptions { RegenerateSpatialWorldIds = true }), Is.True);
            var sources = new HashSet<EntityUid>();
            foreach (var map in new[] { restored!.Value.Owner, clone!.Value.Owner })
            {
                Assert.That(entities.GetComponent<SpatialPhysicsWorldComponent>(map).Settings, Is.EqualTo(settings));
                using var children = entities.GetComponent<TransformComponent>(map).ChildEnumerator;
                Assert.That(children.MoveNext(out var loaded), Is.True);
                Assert.That(children.MoveNext(out _), Is.False);
                var loadedBody = entities.GetComponent<SpatialBodyComponent>(loaded);
                AssertDefinition(loadedBody.Definition, definition);
                Assert.That(loadedBody.DefinitionRevision, Is.EqualTo(1));
                Assert.That(loadedBody.LinearVelocity, Is.EqualTo(velocity));
                Assert.That(loadedBody.AngularVelocity, Is.EqualTo(angular));
                Assert.That(server.System<SharedTransformSystem>().GetWorldPose(loaded), Is.EqualTo(pose));
                Assert.That(physics.GetBodyCount(map), Is.EqualTo(1));
                var hit = physics.GetBackend(map).RayCast(pose.Position + new Vector3(0, 0, 2), -Vector3.UnitZ, 4, 4)!.Value;
                Assert.That(hit.Source.Entity, Is.EqualTo(loaded));
                Assert.That(hit.Source.Revision, Is.EqualTo(1));
                sources.Add(hit.Source.Entity!.Value);
            }
            Assert.That(sources, Has.Count.EqualTo(2));
            using var stream = new MemoryStream();
            var serializer = server.Resolve<IRobustSerializer>();
            serializer.Serialize(stream, new SpatialBodyComponentState(definition, velocity, angular, 7));
            stream.Position = 0;
            var state = serializer.Deserialize<SpatialBodyComponentState>(stream);
            AssertDefinition(state.Definition, definition);
            Assert.That(state.DefinitionRevision, Is.EqualTo(7));
            Assert.That(state.LinearVelocity, Is.EqualTo(velocity));
            Assert.That(state.AngularVelocity, Is.EqualTo(angular));
        });
    }

    [Test]
    public async Task ConnectedClientReceivesShapeAndVelocityWithoutRunningASecondSolver()
    {
        using var server = StartServer(new ServerIntegrationOptions { Pool = false });
        using var client = StartClient(new ClientIntegrationOptions { Pool = false });
        await Task.WhenAll(server.WaitIdleAsync(), client.WaitIdleAsync());
        EntityUid world = default;
        EntityUid body = default;
        NetEntity worldNet = default;
        NetEntity bodyNet = default;
        NetEntity secondBodyNet = default;
        NetEntity constraintNet = default;
        EntityUid part = default;
        NetEntity partNet = default;
        var settings = new SpatialPhysicsSettings(Vector3.Zero, 1, 8);
        await server.WaitAssertion(() =>
        {
            world = server.System<SharedSpatialSystem>().CreateWorld();
            server.System<SharedSpatialPhysicsSystem>().EnablePhysics(world, settings);
            body = SpatialPhysicsTests.SpawnBody(server.EntMan, world, new Vector3(0, 0, 3), Vector3.UnitX);
            part = server.EntMan.CreateEntityUninitialized(null, body,
                new RigidTransform3(Vector3.UnitY * -2, Quaternion.Identity));
            var partComponent = server.EntMan.AddComponent<SpatialBodyComponent>(part);
            server.System<SharedSpatialPhysicsSystem>().ConfigureInitialBody((part, partComponent),
                new SpatialBodyDefinition(SpatialMotionOwner.AnchoredCompoundChild,
                    new[] { new SpatialColliderDefinition(new SpatialSphereShape(0.25f), RigidTransform3.Identity) }, mass: 2));
            server.EntMan.InitializeAndStartEntity(part);
            partNet = server.EntMan.GetNetEntity(part);
            var component = server.EntMan.GetComponent<SpatialBodyComponent>(body);
            server.System<SharedSpatialPhysicsSystem>().ReconfigureBody((body, component), component.Definition,
                0, Vector3.UnitX, Vector3.Zero);
            var second = SpatialPhysicsTests.SpawnBody(server.EntMan, world, new Vector3(0, 2, 3), Vector3.UnitX);
            var constraint = SpatialConstraintTests.SpawnConstraint(server.EntMan, body,
                new SpatialConstraintDefinition(body, second, SpatialConstraintKind.Fixed,
                    relativePose: new RigidTransform3(Vector3.UnitY * 2, Quaternion.Identity)));
            worldNet = server.EntMan.GetNetEntity(world);
            bodyNet = server.EntMan.GetNetEntity(body);
            secondBodyNet = server.EntMan.GetNetEntity(second);
            constraintNet = server.EntMan.GetNetEntity(constraint);
        });
        client.SetConnectTarget(server);
        await client.WaitPost(() => client.Resolve<IClientNetManager>().ClientConnect(null!, 0, null!));
        await RunTicksSync(server, client, 10);
        await server.WaitPost(() =>
        {
            var players = server.Resolve<ISharedPlayerManager>();
            var session = players.Sessions.First();
            server.PlayerMan.SetAttachedEntity(session, body);
            players.JoinGame(session);
        });
        await RunTicksSync(server, client, 20);
        await client.WaitAssertion(() =>
        {
            var entities = client.EntMan;
            var localWorld = entities.GetEntity(worldNet);
            var localBody = entities.GetEntity(bodyNet);
            var physics = client.System<SharedSpatialPhysicsSystem>();
            var component = entities.GetComponent<SpatialBodyComponent>(localBody);
            Assert.That(component.HasDefinition, Is.True);
            Assert.That(component.DefinitionRevision, Is.EqualTo(1));
            Assert.That(entities.GetComponent<SpatialPhysicsWorldComponent>(localWorld).Settings, Is.EqualTo(settings));
            Assert.That(component.Definition.Colliders.Single().Shape, Is.TypeOf<SpatialSphereShape>());
            Assert.That(component.LinearVelocity, Is.EqualTo(Vector3.UnitX));
            var localPart = entities.GetEntity(partNet);
            var partBody = entities.GetComponent<SpatialBodyComponent>(localPart);
            Assert.That(partBody.Definition.Owner, Is.EqualTo(SpatialMotionOwner.AnchoredCompoundChild));
            Assert.That(partBody.Definition.Mass, Is.EqualTo(2));
            Assert.That(partBody.LinearVelocity, Is.EqualTo(Vector3.Zero));
            Assert.That(entities.GetComponent<TransformComponent>(localPart).ParentUid, Is.EqualTo(localBody));
            Assert.That(entities.GetComponent<TransformComponent>(localPart).LocalPose.Position, Is.EqualTo(Vector3.UnitY * -2));
            var before = client.System<SharedTransformSystem>().GetWorldPose(localBody);
            Assert.That(before.Position.Z, Is.EqualTo(3));
            Assert.That(before.Position.X, Is.GreaterThan(0));
            Assert.Throws<InvalidOperationException>(() => physics.SetVelocity((localBody, component), Vector3.UnitY, Vector3.Zero));
            Assert.Throws<InvalidOperationException>(() => physics.ReconfigureBody((localBody, component),
                component.Definition, 1, Vector3.UnitY, Vector3.Zero));
            Assert.Throws<KeyNotFoundException>(() => physics.GetBackend(localWorld));
            var localConstraint = entities.GetEntity(constraintNet);
            var constraint = entities.GetComponent<SpatialConstraintComponent>(localConstraint);
            Assert.That(constraint.HasDefinition, Is.True);
            Assert.That(constraint.Definition.BodyA, Is.EqualTo(localBody));
            Assert.That(constraint.Definition.BodyB, Is.EqualTo(entities.GetEntity(secondBodyNet)));
            Assert.That(constraint.Definition.RelativePose.Position, Is.EqualTo(Vector3.UnitY * 2));
            Assert.That(physics.IsConstraintActive(localConstraint), Is.False);
            Assert.Throws<InvalidOperationException>(() => physics.SetConstraint((localConstraint, constraint), constraint.Definition));
            physics.Update(1);
            Assert.That(client.System<SharedTransformSystem>().GetWorldPose(localBody), Is.EqualTo(before));
        });
        NetEntity destinationNet = default;
        await server.WaitPost(() =>
        {
            var destination = server.System<SharedSpatialSystem>().CreateWorld();
            var physics = server.System<SharedSpatialPhysicsSystem>();
            physics.EnablePhysics(destination, settings);
            physics.TransferGraph(new[] { body, server.EntMan.GetEntity(secondBodyNet) }, destination,
                new RigidTransform3(Vector3.UnitZ * 10, Quaternion.Identity));
            destinationNet = server.EntMan.GetNetEntity(destination);
            server.EntMan.DeleteEntity(world);
            world = destination;
        });
        await RunTicksSync(server, client, 20);
        await client.WaitAssertion(() =>
        {
            var entities = client.EntMan;
            var destination = entities.GetEntity(destinationNet);
            foreach (var net in new[] { bodyNet, secondBodyNet, partNet, constraintNet })
                Assert.That(entities.GetComponent<TransformComponent>(entities.GetEntity(net)).MapUid, Is.EqualTo(destination));
            Assert.That(client.System<SharedTransformSystem>().GetWorldPose(entities.GetEntity(bodyNet)).Position.Z,
                Is.EqualTo(13).Within(1e-4));
            Assert.That(entities.GetComponent<TransformComponent>(entities.GetEntity(partNet)).ParentUid, Is.EqualTo(entities.GetEntity(bodyNet)));
            var d = entities.GetComponent<SpatialConstraintComponent>(entities.GetEntity(constraintNet)).Definition;
            Assert.That(d.BodyA, Is.EqualTo(entities.GetEntity(bodyNet)));
            Assert.That(d.BodyB, Is.EqualTo(entities.GetEntity(secondBodyNet)));
        });
        await server.WaitPost(() => server.EntMan.DeleteEntity(part));
        await RunTicksSync(server, client, 20);
        await client.WaitAssertion(() =>
        {
            Assert.That(client.EntMan.TryGetEntity(partNet, out _), Is.False);
            Assert.That(client.EntMan.EntityExists(client.EntMan.GetEntity(bodyNet)), Is.True);
        });
        await server.WaitPost(() => server.EntMan.DeleteEntity(world));
        await RunTicksSync(server, client, 20);
        await client.WaitAssertion(() => Assert.That(client.EntMan.TryGetEntity(bodyNet, out _), Is.False));
    }

    private static void AssertDefinition(SpatialBodyDefinition actual, SpatialBodyDefinition expected)
    {
        Assert.That(actual.Owner, Is.EqualTo(expected.Owner));
        Assert.That(actual.Mass, Is.EqualTo(expected.Mass));
        Assert.That(actual.Material, Is.EqualTo(expected.Material));
        Assert.That(actual.CollisionLayer, Is.EqualTo(expected.CollisionLayer));
        Assert.That(actual.CollisionMask, Is.EqualTo(expected.CollisionMask));
        Assert.That(actual.Colliders.Select(c => c.Shape.GetType()), Is.EqualTo(expected.Colliders.Select(c => c.Shape.GetType())));
        Assert.That(actual.Colliders.Select(c => c.LocalPose), Is.EqualTo(expected.Colliders.Select(c => c.LocalPose)));
        Assert.That(actual.Colliders.Select(c => c.MassWeight), Is.EqualTo(expected.Colliders.Select(c => c.MassWeight)));
        Assert.That(((SpatialBoxShape) actual.Colliders[0].Shape).Size, Is.EqualTo(new Vector3(0.4f, 0.5f, 0.6f)));
        Assert.That(((SpatialSphereShape) actual.Colliders[1].Shape).Radius, Is.EqualTo(0.2f));
        Assert.That(((SpatialCapsuleShape) actual.Colliders[2].Shape).Length, Is.EqualTo(1.2f));
    }
}
