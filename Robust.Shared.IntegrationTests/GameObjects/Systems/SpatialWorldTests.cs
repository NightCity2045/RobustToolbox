using System.Numerics;
using NUnit.Framework;
using Robust.Shared.GameObjects;
using Robust.Shared.GameStates;
using Robust.Shared.Map.Components;
using Robust.Shared.Maths;
using Robust.Shared.Spatial;
using Robust.UnitTesting.Server;

namespace Robust.UnitTesting.Shared.GameObjects.Systems;

[TestFixture]
public sealed class SpatialWorldTests
{
    [Test]
    public void LayerContextDoesNotMoveEntitiesOrQuantizeTheirHeight()
    {
        var sim = RobustServerSimulation.NewSimulation().InitializeInstance();
        var entities = sim.Resolve<IEntityManager>();
        var spatial = entities.System<SharedSpatialSystem>();
        var transforms = entities.System<SharedTransformSystem>();
        var world = spatial.CreateWorld();
        var first = new LayerId(4);
        var second = new LayerId(900);
        var frame = CreateFrame(entities, world, new Dictionary<LayerId, LayerDefinition>
        {
            [first] = new(3, "Lower", 0.5f),
            [second] = new(7, "Upper", 2),
        });
        transforms.SetLocalPose(frame, new RigidTransform3(new Vector3(10, 20, 30),
            Quaternion.CreateFromAxisAngle(Vector3.UnitX, MathF.PI / 2)));
        var pose = new RigidTransform3(new Vector3(12, 14, 35), Quaternion.CreateFromYawPitchRoll(0.7f, 0.2f, -0.5f));
        var uid = entities.SpawnAttachedTo(null, world, pose);
        var lower = new SpatialLayerReference(frame, first);
        var upper = new SpatialLayerReference(frame, second);
        var moves = 0;
        transforms.OnGlobalMoveEvent += (ref MoveEvent ev) => { if (ev.Sender == uid) moves++; };

        Near(spatial.GetLayerWorldPose(lower).Position, new Vector3(10, 17, 30));
        Near(spatial.GetPoseRelativeToLayer(uid, lower).Position, new Vector3(2, 5, 3));
        Near(spatial.GetPoseRelativeToLayer(uid, upper).Position, new Vector3(2, 5, -1));
        Assert.That(moves, Is.Zero);
        Assert.That(transforms.GetWorldPose(uid), Is.EqualTo(pose));
        Assert.That(entities.GetComponent<TransformComponent>(uid).ParentUid, Is.EqualTo(world.Owner));

        spatial.SetLayerPlanarPosition(uid, lower, new Vector2(-4, 8));
        Near(transforms.GetWorldPose(uid).Position, new Vector3(6, 14, 38));
        Near(spatial.GetPoseRelativeToLayer(uid, lower).Position, new Vector3(-4, 8, 3));
        foreach (var axis in new[] { Vector3.UnitX, Vector3.UnitY, Vector3.UnitZ })
            Near(transforms.GetWorldPose(uid).TransformVector(axis), pose.TransformVector(axis));
        Assert.That(spatial.FromNetwork(spatial.ToNetwork(lower)), Is.EqualTo(lower));

        // Crossing either authoring plane remains an ordinary continuous pose update.
        foreach (var height in new[] { 2.99f, 3.01f, 6.99f, 7.01f })
        {
            var crossing = new RigidTransform3(new Vector3(-1, 2, height), pose.Rotation);
            transforms.SetWorldPose(uid, crossing);
            Assert.That(transforms.GetWorldPose(uid), Is.EqualTo(crossing));
            Assert.That(entities.GetComponent<TransformComponent>(uid).ParentUid, Is.EqualTo(world.Owner));
        }
    }

    [Test]
    public void WorldIdentityAndFrameMembershipFollowTransferAndDeletion()
    {
        var sim = RobustServerSimulation.NewSimulation().InitializeInstance();
        var entities = sim.Resolve<IEntityManager>();
        var spatial = entities.System<SharedSpatialSystem>();
        var transforms = entities.System<SharedTransformSystem>();
        var first = spatial.CreateWorld();
        var second = spatial.CreateWorld();
        var firstId = first.Comp.PersistentId;
        var secondId = second.Comp.PersistentId;
        Assert.That(firstId.IsValid, Is.True);
        Assert.That(secondId, Is.Not.EqualTo(firstId));
        var layer = new LayerId(7);
        var frame = CreateFrame(entities, first, new Dictionary<LayerId, LayerDefinition> { [layer] = new(-2.5f, "Basement") });
        var reference = new SpatialLayerReference(frame, layer);
        var uid = entities.SpawnAttachedTo(null, second, RigidTransform3.Identity);
        Assert.Throws<InvalidOperationException>(() => spatial.GetPoseRelativeToLayer(uid, reference));
        Assert.Throws<InvalidOperationException>(() => spatial.SetLayerPlanarPosition(uid, reference, Vector2.One));
        Assert.That(transforms.GetWorldPose(uid), Is.EqualTo(RigidTransform3.Identity));

        transforms.SetParent(frame, second);
        Assert.That(spatial.GetWorld(frame).Owner, Is.EqualTo(second.Owner));
        Near(spatial.GetPoseRelativeToLayer(uid, reference).Position, new Vector3(0, 0, 2.5f));
        entities.DeleteEntity(first);
        Assert.That(spatial.TryResolveWorld(firstId, out _), Is.False);
        Assert.That(spatial.TryResolveWorld(secondId, out var resolved), Is.True);
        Assert.That(resolved, Is.EqualTo(second.Owner));
        Assert.That(entities.EntityExists(frame), Is.True);
        entities.DeleteEntity(second);
        Assert.That(entities.Deleted(frame), Is.True);
        Assert.That(spatial.TryResolveWorld(secondId, out _), Is.False);
    }

    [Test]
    public void InvalidDefinitionsAndStatesDoNotPartiallyReplaceExistingData()
    {
        var sim = RobustServerSimulation.NewSimulation().InitializeInstance();
        var entities = sim.Resolve<IEntityManager>();
        var spatial = entities.System<SharedSpatialSystem>();
        var world = spatial.CreateWorld();
        var otherWorld = spatial.CreateWorld();
        var id = world.Comp.PersistentId;
        var key = new LayerId(12);
        var valid = new LayerDefinition(-1.75f, "Mezzanine", 0.25f);
        var definitions = new Dictionary<LayerId, LayerDefinition> { [key] = valid };
        var frame = CreateFrame(entities, world, definitions);
        var component = entities.GetComponent<SpatialFrameComponent>(frame);
        definitions.Clear();
        Assert.That(component.Layers[key], Is.EqualTo(valid));
        Assert.Throws<InvalidOperationException>(() => spatial.ConfigureInitialLayers((frame, component), definitions));
        Assert.Throws<NotSupportedException>(() => ((IDictionary<LayerId, LayerDefinition>) component.Layers).Clear());

        var badLayers = new ComponentHandleState(new SpatialFrameComponentState(
            new Dictionary<LayerId, LayerDefinition> { [new LayerId(13)] = default }), null);
        Assert.Throws<ArgumentOutOfRangeException>(() => spatial.OnFrameHandleState(frame, component, ref badLayers));
        Assert.That(component.Layers, Has.Count.EqualTo(1));
        Assert.That(component.Layers[key], Is.EqualTo(valid));

        foreach (var invalid in new[]
                 {
                     new SpatialWorldComponentState(default, SpatialWorldComponent.CurrentDefinitionSchema),
                     new SpatialWorldComponentState(otherWorld.Comp.PersistentId, SpatialWorldComponent.CurrentDefinitionSchema),
                     new SpatialWorldComponentState(PersistentWorldId.New(), 999),
                 })
        {
            var state = new ComponentHandleState(invalid, null);
            Assert.Throws<InvalidOperationException>(() => spatial.OnWorldHandleState(world, world.Comp, ref state));
            Assert.That(world.Comp.PersistentId, Is.EqualTo(id));
            Assert.That(spatial.TryResolveWorld(id, out var registered), Is.True);
            Assert.That(registered, Is.EqualTo(world.Owner));
        }
    }

    internal static EntityUid CreateFrame(IEntityManager entities, EntityUid world,
        Dictionary<LayerId, LayerDefinition> definitions)
    {
        var frame = entities.CreateEntityUninitialized(null, world, RigidTransform3.Identity);
        var component = entities.AddComponent<SpatialFrameComponent>(frame);
        entities.System<SharedSpatialSystem>().ConfigureInitialLayers((frame, component), definitions);
        entities.InitializeAndStartEntity(frame, entities.GetComponent<MapComponent>(world).MapId);
        return frame;
    }

    private static void Near(Vector3 actual, Vector3 expected)
        => Assert.That(Vector3.Distance(actual, expected), Is.LessThan(0.0001f));
}
