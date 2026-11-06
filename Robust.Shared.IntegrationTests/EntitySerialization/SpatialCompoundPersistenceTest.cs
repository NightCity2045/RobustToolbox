using System.Numerics;
using NUnit.Framework;
using Robust.Shared.EntitySerialization;
using Robust.Shared.EntitySerialization.Systems;
using Robust.Shared.GameObjects;
using Robust.Shared.Map.Components;
using Robust.Shared.Maths;
using Robust.Shared.Spatial;
using Robust.Shared.Spatial.Physics;
using Robust.Shared.Utility;
using Robust.UnitTesting.Shared.GameObjects.Systems;

namespace Robust.UnitTesting.Shared.EntitySerialization;

[TestFixture]
public sealed class SpatialCompoundPersistenceTest : RobustIntegrationTest
{
    [Test]
    public async Task SavedAndClonedCompoundRebuildsMassSourcesAndSeatConnection()
    {
        using var server = StartServer(new ServerIntegrationOptions { Pool = false });
        await server.WaitIdleAsync();
        await server.WaitAssertion(() =>
        {
            var entities = server.EntMan;
            var physics = server.System<SharedSpatialPhysicsSystem>();
            var transforms = server.System<SharedTransformSystem>();
            var world = server.System<SharedSpatialSystem>().CreateWorld();
            physics.EnablePhysics(world, new SpatialPhysicsSettings(Vector3.Zero, 1, 8));
            var root = Spawn(world, new Vector3(10, 20, 30), SpatialMotionOwner.DynamicBody, 2);
            var seat = Spawn(root, Vector3.UnitX * 3, SpatialMotionOwner.AnchoredCompoundChild, 4);
            var passenger = Spawn(root, new Vector3(3, 0, 2), SpatialMotionOwner.DynamicBody, 3);
            SpatialConstraintTests.SpawnConstraint(entities, world, new SpatialConstraintDefinition(seat, passenger,
                SpatialConstraintKind.Fixed, relativePose: new RigidTransform3(Vector3.UnitZ * 2, Quaternion.Identity)));
            physics.Update(1f / 30);
            var path = new ResPath("/spatial-compound.yml");
            var loader = server.System<MapLoaderSystem>();
            Assert.That(loader.TrySaveMap(entities.GetComponent<MapComponent>(world).MapId, path), Is.True);
            entities.DeleteEntity(world);
            Assert.That(loader.TryLoadMap(path, out var restored, out _), Is.True);
            Assert.That(loader.TryLoadMap(path, out var clone, out _,
                new DeserializationOptions { RegenerateSpatialWorldIds = true }), Is.True);
            var identities = new HashSet<EntityUid>();
            foreach (var map in new[] { restored!.Value.Owner, clone!.Value.Owner })
            {
                var children = entities.GetComponent<TransformComponent>(map)._children;
                var loadedRoot = children.Single(entities.HasComponent<SpatialBodyComponent>);
                var loadedLink = children.Single(entities.HasComponent<SpatialConstraintComponent>);
                var parts = entities.GetComponent<TransformComponent>(loadedRoot)._children;
                var loadedSeat = parts.Single(uid => entities.GetComponent<SpatialBodyComponent>(uid).Definition.Owner
                    == SpatialMotionOwner.AnchoredCompoundChild);
                var loadedPassenger = parts.Single(uid => uid != loadedSeat);
                foreach (var uid in new[] { loadedRoot, loadedSeat, loadedPassenger, loadedLink })
                    Assert.That(identities.Add(uid), Is.True, "Cloned graphs must have distinct canonical entities.");
                Assert.That(physics.GetBodyCount(map), Is.EqualTo(2), "The seat is part of the root, not a third solver body.");
                var seatBody = entities.GetComponent<SpatialBodyComponent>(loadedSeat);
                Assert.That(seatBody.LinearVelocity, Is.EqualTo(Vector3.Zero));
                var seatPose = transforms.GetWorldPose(loadedSeat);
                var hit = server.System<SpatialQuerySystem>().RayCast(new SpatialQueryContext(map, map, "test", uint.MaxValue),
                    seatPose.Position + Vector3.UnitY * 2, -Vector3.UnitY, 3);
                Assert.That(hit!.Value.Entity, Is.EqualTo(loadedSeat), "Derived collider identity must use the remapped part.");
                var constraint = entities.GetComponent<SpatialConstraintComponent>(loadedLink);
                Assert.That(constraint.Definition.BodyA, Is.EqualTo(loadedSeat));
                Assert.That(constraint.Definition.BodyB, Is.EqualTo(loadedPassenger));
                physics.Update(1f / 30);
                Assert.That(physics.IsConstraintActive(loadedLink), Is.True);
                physics.RemoveConstraint((loadedLink, constraint));
                var rootBody = entities.GetComponent<SpatialBodyComponent>(loadedRoot);
                var origin = transforms.GetWorldPose(loadedRoot).Position;
                var center = (origin * 2 + transforms.GetWorldPose(loadedSeat).Position * 4) / 6;
                physics.ApplyImpulse((loadedSeat, seatBody), Vector3.UnitY * 6, center);
                Assert.That(Vector3.Distance(rootBody.LinearVelocity, Vector3.UnitY), Is.LessThan(1e-4));
                Assert.That(rootBody.AngularVelocity.Length(), Is.LessThan(1e-4));
                entities.DeleteEntity(loadedSeat);
                physics.ApplyImpulse((loadedRoot, rootBody), Vector3.UnitY * 2, origin);
                Assert.That(Vector3.Distance(rootBody.LinearVelocity, Vector3.UnitY * 2), Is.LessThan(1e-4));
            }

            EntityUid Spawn(EntityUid parent, Vector3 position, SpatialMotionOwner owner, float mass)
            {
                var uid = entities.CreateEntityUninitialized(null, parent, new RigidTransform3(position, Quaternion.Identity));
                var body = entities.AddComponent<SpatialBodyComponent>(uid);
                physics.ConfigureInitialBody((uid, body), new SpatialBodyDefinition(owner,
                    new[] { new SpatialColliderDefinition(new SpatialSphereShape(0.25f), RigidTransform3.Identity) }, mass: mass));
                entities.InitializeAndStartEntity(uid);
                return uid;
            }
        });
    }
}
