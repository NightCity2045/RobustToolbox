using System.Numerics;
using NUnit.Framework;
using Robust.Shared.Containers;
using Robust.Shared.GameObjects;
using Robust.Shared.Map.Components;
using Robust.Shared.Maths;
using Robust.Shared.Spatial;
using Robust.Shared.Spatial.Physics;
using Robust.UnitTesting.Server;

namespace Robust.UnitTesting.Shared.GameObjects.Systems;

[TestFixture]
public sealed class SpatialConstraintTests
{
    [Test]
    public void ConnectionsReconfigureAtomicallySuspendInContainersAndReleaseDeletedBodies()
    {
        var sim = RobustServerSimulation.NewSimulation().InitializeInstance();
        var entities = sim.Resolve<IEntityManager>();
        var physics = entities.System<SharedSpatialPhysicsSystem>();
        var world = entities.System<SharedSpatialSystem>().CreateWorld();
        physics.EnablePhysics(world, new SpatialPhysicsSettings(Vector3.Zero, 1, 8));
        var a = SpatialPhysicsTests.SpawnBody(entities, world, Vector3.Zero);
        var b = SpatialPhysicsTests.SpawnBody(entities, world, Vector3.UnitX * 2);
        var constraint = SpawnConstraint(entities, world, new SpatialConstraintDefinition(a, b, SpatialConstraintKind.Fixed,
            relativePose: new RigidTransform3(Vector3.UnitX * 2, Quaternion.Identity)));
        physics.Update(1f / 30);
        Assert.That(physics.IsConstraintActive(constraint), Is.True);
        var component = entities.GetComponent<SpatialConstraintComponent>(constraint);
        var original = component.Definition;
        var otherWorld = entities.System<SharedSpatialSystem>().CreateWorld();
        physics.EnablePhysics(otherWorld);
        var foreign = SpatialPhysicsTests.SpawnBody(entities, otherWorld, Vector3.Zero);
        Assert.Throws<InvalidOperationException>(() => physics.SetConstraint((constraint, component),
            new SpatialConstraintDefinition(a, foreign, SpatialConstraintKind.Point)));
        Assert.That(component.Definition, Is.SameAs(original));
        Assert.That(physics.IsConstraintActive(constraint), Is.True);
        physics.SetConstraint((constraint, component), new SpatialConstraintDefinition(a, b, SpatialConstraintKind.DistanceRange,
            minimumDistance: 1, maximumDistance: 3));
        Assert.That(component.Definition.Kind, Is.EqualTo(SpatialConstraintKind.DistanceRange));
        Assert.That(physics.IsConstraintActive(constraint), Is.True);
        var holder = entities.SpawnAttachedTo(null, world, RigidTransform3.Identity);
        var containers = entities.System<SharedContainerSystem>();
        var container = containers.EnsureContainer<Container>(holder, "storage");
        Assert.That(containers.Insert(a, container), Is.True);
        physics.Update(1f / 30);
        Assert.That(physics.IsConstraintActive(constraint), Is.False);
        Assert.That(component.Definition.BodyA, Is.EqualTo(a));
        Assert.That(containers.Remove(a, container), Is.True);
        physics.Update(1f / 30);
        Assert.That(physics.IsConstraintActive(constraint), Is.True);
        entities.System<MetaDataSystem>().SetEntityPaused(constraint, true);
        physics.Update(1f / 30);
        Assert.That(physics.IsConstraintActive(constraint), Is.False);
        entities.System<MetaDataSystem>().SetEntityPaused(constraint, false);
        physics.Update(1f / 30);
        Assert.That(physics.IsConstraintActive(constraint), Is.True);
        entities.DeleteEntity(a);
        Assert.That(physics.IsConstraintActive(constraint), Is.False);
        Assert.That(entities.HasComponent<SpatialConstraintComponent>(constraint), Is.False);
        Assert.That(entities.EntityExists(b), Is.True);
        physics.Update(1f / 30);
    }

    [Test]
    public void WholeSubgraphTransferRebuildsConstraintsInDestinationWorld()
    {
        var sim = RobustServerSimulation.NewSimulation().InitializeInstance();
        var entities = sim.Resolve<IEntityManager>();
        var physics = entities.System<SharedSpatialPhysicsSystem>();
        var spatial = entities.System<SharedSpatialSystem>();
        var first = spatial.CreateWorld();
        var second = spatial.CreateWorld();
        var settings = new SpatialPhysicsSettings(Vector3.Zero, 1, 8);
        physics.EnablePhysics(first, settings);
        physics.EnablePhysics(second, settings);
        var group = entities.SpawnAttachedTo(null, first, RigidTransform3.Identity);
        var a = SpatialPhysicsTests.SpawnBody(entities, group, Vector3.Zero);
        var b = SpatialPhysicsTests.SpawnBody(entities, group, Vector3.UnitX * 2);
        var link = SpawnConstraint(entities, group, new SpatialConstraintDefinition(a, b, SpatialConstraintKind.Fixed,
            relativePose: new RigidTransform3(Vector3.UnitX * 2, Quaternion.Identity)));
        physics.Update(1f / 30);
        entities.System<SharedTransformSystem>().SetParent(group, second);
        physics.Update(1f / 30);
        Assert.That(physics.GetBodyCount(first), Is.Zero);
        Assert.That(physics.GetBodyCount(second), Is.EqualTo(2));
        Assert.That(physics.IsConstraintActive(link), Is.True);
        physics.ApplyImpulse((a, entities.GetComponent<SpatialBodyComponent>(a)), Vector3.UnitY * 2, Vector3.Zero);
        for (var i = 0; i < 60; i++)
            physics.Update(1f / 30);
        var transforms = entities.System<SharedTransformSystem>();
        var relative = transforms.GetWorldPose(a).ToLocal(transforms.GetWorldPose(b));
        Assert.That(Vector3.Distance(relative.Position, Vector3.UnitX * 2), Is.LessThan(0.01));
        entities.DeleteEntity(first);
        Assert.That(physics.IsConstraintActive(link), Is.True);
        entities.DeleteEntity(second);
        Assert.That(physics.IsConstraintActive(link), Is.False);
    }

    internal static EntityUid SpawnConstraint(IEntityManager entities, EntityUid parent, SpatialConstraintDefinition definition)
    {
        var uid = entities.CreateEntityUninitialized(null, parent, RigidTransform3.Identity);
        var component = entities.AddComponent<SpatialConstraintComponent>(uid);
        entities.System<SharedSpatialPhysicsSystem>().ConfigureInitialConstraint((uid, component), definition);
        entities.InitializeAndStartEntity(uid, entities.GetComponent<TransformComponent>(parent).MapID);
        return uid;
    }
}
