using System.Numerics;
using NUnit.Framework;
using Robust.Shared.Containers;
using Robust.Shared.GameObjects;
using Robust.Shared.Maths;
using Robust.Shared.Spatial;
using Robust.Shared.Spatial.Physics;
using Robust.UnitTesting.Server;

namespace Robust.UnitTesting.Shared.GameObjects.Systems;

[TestFixture]
public sealed class SpatialBodyReconfigureTests
{
    private static SpatialBodyDefinition Sphere(float radius, SpatialMotionOwner owner = SpatialMotionOwner.DynamicBody)
        => new(owner, new[] { new SpatialColliderDefinition(new SpatialSphereShape(radius), RigidTransform3.Identity) }, mass: 2);

    [Test]
    public void ReplacementPublishesRevisionAndFailurePreservesTheOriginalBody()
    {
        var sim = RobustServerSimulation.NewSimulation().InitializeInstance();
        var entities = sim.Resolve<IEntityManager>();
        var physics = entities.System<SharedSpatialPhysicsSystem>();
        var world = entities.System<SharedSpatialSystem>().CreateWorld();
        physics.EnablePhysics(world, new SpatialPhysicsSettings(Vector3.Zero, 1, 8));
        var uid = SpatialPhysicsTests.SpawnBody(entities, world, new Vector3(0, 0, 3));
        Entity<SpatialBodyComponent> body = (uid, entities.GetComponent<SpatialBodyComponent>(uid));
        physics.ReconfigureBody(body, Sphere(0.5f), 0, Vector3.UnitX * 2, Vector3.Zero);
        Assert.That(body.Comp.DefinitionRevision, Is.EqualTo(1));
        Assert.That(body.Comp.LinearVelocity, Is.EqualTo(Vector3.UnitX * 2));
        var definition = body.Comp.Definition;
        var backend = physics.GetBackend(world);
        var hit = backend.RayCast(new Vector3(0, 0, 6), -Vector3.UnitZ, 6, uint.MaxValue)!.Value;
        Assert.That(hit.Distance, Is.EqualTo(2.5f).Within(1e-5));
        Assert.That(hit.Source.Revision, Is.EqualTo(1));
        Assert.Throws<InvalidOperationException>(() => physics.ReconfigureBody(body, Sphere(1), 0, Vector3.Zero, Vector3.Zero));
        // This is a finite positive shape, but its inertia cannot be represented.
        Assert.Throws<ArgumentException>(() => physics.ReconfigureBody(body, Sphere(1e-25f), 1, Vector3.Zero, Vector3.Zero));
        Assert.That(body.Comp.Definition, Is.SameAs(definition));
        Assert.That(body.Comp.DefinitionRevision, Is.EqualTo(1));
        Assert.That(body.Comp.LinearVelocity, Is.EqualTo(Vector3.UnitX * 2));
        Assert.That(physics.GetBodyCount(world), Is.EqualTo(1));
        Assert.That(backend.RayCast(new Vector3(0, 0, 6), -Vector3.UnitZ, 6, uint.MaxValue)!.Value.Body, Is.EqualTo(hit.Body));
    }

    [Test]
    public void ContainedBodyCanChangeDefinitionWithoutEnteringThePhysicalScene()
    {
        var sim = RobustServerSimulation.NewSimulation().InitializeInstance();
        var entities = sim.Resolve<IEntityManager>();
        var physics = entities.System<SharedSpatialPhysicsSystem>();
        var world = entities.System<SharedSpatialSystem>().CreateWorld();
        physics.EnablePhysics(world, new SpatialPhysicsSettings(Vector3.Zero, 1, 8));
        var uid = SpatialPhysicsTests.SpawnBody(entities, world, Vector3.Zero);
        Entity<SpatialBodyComponent> body = (uid, entities.GetComponent<SpatialBodyComponent>(uid));
        var holder = entities.SpawnAttachedTo(null, world, RigidTransform3.Identity);
        var containers = entities.System<SharedContainerSystem>();
        var container = containers.EnsureContainer<Container>(holder, "storage");
        Assert.That(containers.Insert(uid, container), Is.True);
        Assert.That(physics.GetBodyCount(world), Is.Zero);
        physics.ReconfigureBody(body, Sphere(0.5f), 0, Vector3.UnitY * 2, Vector3.Zero);
        Assert.That(physics.GetBodyCount(world), Is.Zero);
        Assert.That(body.Comp.DefinitionRevision, Is.EqualTo(1));
        Assert.Throws<ArgumentException>(() => physics.ReconfigureBody(body, Sphere(1e-25f), 1, Vector3.Zero, Vector3.Zero));
        Assert.That(body.Comp.DefinitionRevision, Is.EqualTo(1));
        Assert.That(containers.Remove(uid, container), Is.True);
        Assert.That(physics.GetBodyCount(world), Is.EqualTo(1));
        var hit = physics.GetBackend(world).RayCast(Vector3.UnitZ * 3, -Vector3.UnitZ, 4, uint.MaxValue)!.Value;
        Assert.That(hit.Distance, Is.EqualTo(2.5f).Within(1e-5));
        Assert.That(hit.Source.Revision, Is.EqualTo(1));
        physics.Update(1f / 30);
        Assert.That(entities.System<SharedTransformSystem>().GetWorldPose(uid).Position.Y, Is.EqualTo(2f / 30).Within(1e-5));
    }

    [Test]
    public void DefinitionChangePreservesCanonicalConnectionsAndRejectsInvalidOwnership()
    {
        var sim = RobustServerSimulation.NewSimulation().InitializeInstance();
        var entities = sim.Resolve<IEntityManager>();
        var physics = entities.System<SharedSpatialPhysicsSystem>();
        var world = entities.System<SharedSpatialSystem>().CreateWorld();
        physics.EnablePhysics(world, new SpatialPhysicsSettings(Vector3.Zero, 1, 8));
        var a = SpatialPhysicsTests.SpawnBody(entities, world, Vector3.Zero);
        Entity<SpatialBodyComponent> first = (a, entities.GetComponent<SpatialBodyComponent>(a));
        physics.ReconfigureBody(first, Sphere(0.25f, SpatialMotionOwner.Static), 0, Vector3.Zero, Vector3.Zero);
        var b = SpatialPhysicsTests.SpawnBody(entities, world, Vector3.UnitX * 2);
        Entity<SpatialBodyComponent> second = (b, entities.GetComponent<SpatialBodyComponent>(b));
        var link = SpatialConstraintTests.SpawnConstraint(entities, world, new SpatialConstraintDefinition(a, b,
            SpatialConstraintKind.Fixed, relativePose: new RigidTransform3(Vector3.UnitX * 2, Quaternion.Identity)));
        physics.Update(1f / 30);
        Assert.Throws<InvalidOperationException>(() => physics.ReconfigureBody(second, Sphere(0.25f, SpatialMotionOwner.Static),
            0, Vector3.Zero, Vector3.Zero));
        Assert.That(physics.IsConstraintActive(link), Is.True);
        Assert.That(second.Comp.DefinitionRevision, Is.Zero);
        physics.ReconfigureBody(second, Sphere(0.5f), 0, Vector3.Zero, Vector3.Zero);
        Assert.That(physics.IsConstraintActive(link), Is.False);
        physics.Update(1f / 30);
        Assert.That(physics.IsConstraintActive(link), Is.True);
        Assert.That(physics.GetBodyCount(world), Is.EqualTo(2));
    }
}
