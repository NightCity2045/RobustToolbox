using System.Numerics;
using NUnit.Framework;
using Robust.Shared.GameObjects;
using Robust.Shared.Map.Components;
using Robust.Shared.Maths;
using Robust.Shared.Reflection;
using Robust.Shared.Spatial;
using Robust.Shared.Spatial.Physics;
using Robust.UnitTesting.Server;

namespace Robust.UnitTesting.Shared.GameObjects.Systems;

[TestFixture]
public sealed class SpatialGeometryTransactionTests
{
    internal static EntityUid Frame(IEntityManager entities, EntityUid world, Vector3 position = default,
        bool dynamic = false, bool tiles = false)
    {
        var uid = entities.CreateEntityUninitialized(null, world, new RigidTransform3(position, Quaternion.Identity));
        var frame = entities.AddComponent<SpatialFrameComponent>(uid);
        var layer = new LayerId(1);
        entities.System<SharedSpatialSystem>().ConfigureInitialLayers((uid, frame),
            new Dictionary<LayerId, LayerDefinition> { [layer] = new(0, "Floor") });
        var geometry = entities.AddComponent<SpatialGeometryComponent>(uid);
        var surface = new SpatialSurfaceReference(uid, new SurfaceId(1));
        entities.System<SharedGeometrySystem>().ConfigureInitialGeometry((uid, geometry), Array.Empty<BoundarySegment>(),
            new[] { new SlabSurface(surface, layer, new Box2(-2, -2, 2, 2), 0, 0.2f) },
            tileChunks: tiles ? new[] { new SpatialTileChunk(new SpatialChunkKey(layer, Vector2i.Zero),
                new Dictionary<Vector2i, SpatialFloorTile> { [Vector2i.Zero] = new("steel", surface.Surface) }) } : null);
        if (dynamic)
        {
            var body = entities.AddComponent<SpatialBodyComponent>(uid);
            entities.System<SharedSpatialPhysicsSystem>().ConfigureInitialBody((uid, body),
                new SpatialBodyDefinition(SpatialMotionOwner.DynamicBody, useGeometry: true));
        }
        entities.InitializeAndStartEntity(uid, entities.GetComponent<MapComponent>(world).MapId);
        return uid;
    }

    private static SpatialGeometryEdit Empty(SharedGeometrySystem geometry, EntityUid frame)
        => new(geometry.CaptureSnapshot(frame), boundaries: Array.Empty<BoundarySegment>(), slabs: Array.Empty<SlabSurface>(),
            tileChunks: Array.Empty<SpatialTileChunk>());

    private static SpatialGeometryEdit Raised(SharedGeometrySystem geometry, EntityUid frame, float height)
    {
        var before = geometry.CaptureSnapshot(frame);
        var slab = before.Slabs.Values.Single();
        return new SpatialGeometryEdit(before, slabs: new[] { new SlabSurface(slab.Surface, slab.Layer, slab.Footprint,
            height, slab.Thickness, materialProfile: slab.MaterialProfile, entity: slab.Entity) });
    }

    private static (IEntityManager Entities, SharedGeometrySystem Geometry, SharedSpatialPhysicsSystem Physics, EntityUid World)
        Scene(bool enablePhysics = true)
    {
        var sim = RobustServerSimulation.NewSimulation()
            .RegisterEntitySystems(f => f.LoadExtraSystemType<EditObserver>()).InitializeInstance();
        var entities = sim.Resolve<IEntityManager>();
        var physics = entities.System<SharedSpatialPhysicsSystem>();
        var world = entities.System<SharedSpatialSystem>().CreateWorld();
        if (enablePhysics) physics.EnablePhysics(world);
        return (entities, entities.System<SharedGeometrySystem>(), physics, world);
    }

    [Test]
    public void MultiFrameCommitInstallsCollisionsBeforeEventsAndDropsSupportInSameTick()
    {
        var (entities, geometry, physics, world) = Scene();
        var a = Frame(entities, world);
        var b = Frame(entities, world, Vector3.UnitX * 10);
        var motor = SpatialMotorTests.SpawnMotor(entities, world, Vector3.UnitZ * 0.8f);
        for (var i = 0; i < 30; i++) physics.Update(1f / 60);
        var before = geometry.CaptureSnapshot(a);
        using var transaction = geometry.PrepareTransaction(new[] { Empty(geometry, a), Raised(geometry, b, 2) });
        Assert.That(physics.GetBodyCount(world), Is.EqualTo(3));
        Assert.That(geometry.IsSnapshotCurrent(before), Is.True);
        Assert.That(motor.Comp.Support, Is.Not.Null);
        var observer = entities.System<EditObserver>();
        observer.Completed = tx =>
        {
            Assert.That(tx, Is.SameAs(transaction));
            Assert.That(tx.Status, Is.EqualTo(WorldEditStatus.Committed));
            Assert.That(geometry.CaptureSnapshot(a).SolidParts, Is.Empty);
            Assert.That(geometry.CaptureSnapshot(b).Revision, Is.EqualTo(1));
            Assert.That(physics.GetBackend(world).RayCast(new Vector3(1, 0, 3), -Vector3.UnitZ, 4, uint.MaxValue), Is.Null);
            var hit = physics.GetBackend(world).RayCast(new Vector3(10, 0, 3), -Vector3.UnitZ, 4, uint.MaxValue)!.Value;
            Assert.That(hit.Point.Z, Is.EqualTo(2).Within(1e-5));
            Assert.That(hit.Source.Revision, Is.EqualTo(1));
            Assert.Throws<InvalidOperationException>(() => physics.Update(1f / 60));
        };
        geometry.QueueTransaction(transaction);
        Assert.That(geometry.CaptureSnapshot(a).SolidParts, Has.Count.EqualTo(1));
        physics.Update(1f / 60);
        Assert.That(observer.Count, Is.EqualTo(1));
        Assert.That(motor.Comp.Support, Is.Null);
        Assert.That(entities.GetComponent<SpatialBodyComponent>(motor).LinearVelocity.Z, Is.LessThan(0));
        Assert.That(geometry.IsSnapshotCurrent(before), Is.False);
        Assert.That(before.SolidParts, Has.Count.EqualTo(1));
        Assert.That(transaction.Changes.All(c => c.Surfaces.SequenceEqual(new[] { new SurfaceId(1) })), Is.True);
    }

    [Test]
    public void ConflictRejectsEveryFrameAndInverseRequiresTheCommittedRevision()
    {
        var (entities, geometry, physics, world) = Scene();
        var a = Frame(entities, world);
        var b = Frame(entities, world, Vector3.UnitX * 10);
        using var winner = geometry.PrepareTransaction(new[] { Empty(geometry, a) });
        using var stale = geometry.PrepareTransaction(new[] { Raised(geometry, a, 1), Raised(geometry, b, 2) });
        geometry.QueueTransaction(winner);
        geometry.QueueTransaction(stale);
        physics.Update(1f / 60);
        Assert.That(winner.Status, Is.EqualTo(WorldEditStatus.Committed));
        Assert.That(stale.Status, Is.EqualTo(WorldEditStatus.Rejected));
        Assert.That(geometry.CaptureSnapshot(a).SolidParts, Is.Empty);
        Assert.That(geometry.CaptureSnapshot(b).Revision, Is.Zero);
        Assert.That(physics.GetBodyCount(world), Is.EqualTo(1));
        using var undo = geometry.PrepareTransaction(winner.CreateInverse());
        geometry.QueueTransaction(undo);
        physics.Update(1f / 60);
        Assert.That(geometry.CaptureSnapshot(a).Revision, Is.EqualTo(2));
        var hit = physics.GetBackend(world).RayCast(new Vector3(0, 0, 3), -Vector3.UnitZ, 4, uint.MaxValue)!.Value;
        Assert.That(hit.Source.Revision, Is.EqualTo(2));
        Assert.Throws<InvalidOperationException>(() => geometry.PrepareTransaction(winner.CreateInverse()));
    }

    [Test]
    public void PhysicalDefinitionOrWorldActivationChangeInvalidatesPreparation()
    {
        var (entities, geometry, physics, world) = Scene();
        var frame = Frame(entities, world, dynamic: true);
        using var transaction = geometry.PrepareTransaction(new[] { Raised(geometry, frame, 1) });
        var body = entities.GetComponent<SpatialBodyComponent>(frame);
        physics.ReconfigureBody((frame, body), new SpatialBodyDefinition(SpatialMotionOwner.DynamicBody, useGeometry: true, mass: 2),
            0, Vector3.Zero, Vector3.Zero);
        geometry.QueueTransaction(transaction);
        physics.Update(1f / 60);
        Assert.That(transaction.Status, Is.EqualTo(WorldEditStatus.Rejected));
        Assert.That(geometry.CaptureSnapshot(frame).Revision, Is.Zero);
        Assert.That(physics.GetBodyCount(world), Is.EqualTo(1));

        var (otherEntities, otherGeometry, otherPhysics, otherWorld) = Scene(false);
        var authoring = Frame(otherEntities, otherWorld);
        using var edit = otherGeometry.PrepareTransaction(new[] { Raised(otherGeometry, authoring, 2) });
        otherPhysics.EnablePhysics(otherWorld);
        otherGeometry.QueueTransaction(edit);
        otherPhysics.Update(1f / 60);
        Assert.That(edit.Status, Is.EqualTo(WorldEditStatus.Rejected));
        Assert.That(otherGeometry.CaptureSnapshot(authoring).Revision, Is.Zero);
    }

    [Test]
    public void CancellationDeletionAndComponentReplacementRejectOldWorkWithoutLeakingBodies()
    {
        var (entities, geometry, physics, world) = Scene();
        var frame = Frame(entities, world);
        var before = geometry.CaptureSnapshot(frame);
        using (var cancelled = geometry.PrepareTransaction(new[] { Raised(geometry, frame, 1) }))
        {
            geometry.QueueTransaction(cancelled);
            cancelled.Dispose();
            physics.Update(1f / 60);
            Assert.That(cancelled.Status, Is.EqualTo(WorldEditStatus.Cancelled));
            Assert.That(physics.GetBodyCount(world), Is.EqualTo(1));
        }
        using var deleted = geometry.PrepareTransaction(new[] { Raised(geometry, frame, 2) });
        geometry.QueueTransaction(deleted);
        entities.RemoveComponent<SpatialGeometryComponent>(frame);
        Assert.That(deleted.Status, Is.EqualTo(WorldEditStatus.Rejected));
        entities.AddComponent<SpatialGeometryComponent>(frame);
        Assert.That(geometry.IsSnapshotCurrent(before), Is.False);
        Assert.Throws<InvalidOperationException>(() => geometry.PrepareTransaction(new[] { new SpatialGeometryEdit(before) }));
        physics.Update(1f / 60);
        Assert.That(physics.GetBodyCount(world), Is.Zero);
        using var pending = geometry.PrepareTransaction(new[] { new SpatialGeometryEdit(geometry.CaptureSnapshot(frame)) });
        geometry.QueueTransaction(pending);
        entities.DeleteEntity(world);
        Assert.That(pending.Status, Is.EqualTo(WorldEditStatus.Rejected));
        Assert.DoesNotThrow(() => physics.Update(1f / 60));
    }

    [Test]
    public void DecorativeEditKeepsColliderIdentityAndInvalidatesOnlyItsChunk()
    {
        var (entities, geometry, physics, world) = Scene();
        var frame = Frame(entities, world, tiles: true);
        var before = geometry.CaptureSnapshot(frame);
        var original = physics.GetBackend(world).RayCast(Vector3.UnitZ * 3, -Vector3.UnitZ, 4, uint.MaxValue)!.Value;
        var chunk = before.TileChunks.Values.Single();
        var changed = new SpatialTileChunk(chunk.Key,
            new Dictionary<Vector2i, SpatialFloorTile> { [Vector2i.Zero] = new("wood", new SurfaceId(1)) });
        using var edit = geometry.PrepareTransaction(new[] { new SpatialGeometryEdit(before, tileChunks: new[] { changed }) });
        geometry.QueueTransaction(edit);
        physics.Update(1f / 60);
        var after = geometry.CaptureSnapshot(frame);
        var hit = physics.GetBackend(world).RayCast(Vector3.UnitZ * 3, -Vector3.UnitZ, 4, uint.MaxValue)!.Value;
        Assert.That(hit.Body, Is.EqualTo(original.Body));
        Assert.That(hit.Source.Revision, Is.Zero);
        Assert.That(after.Revision, Is.EqualTo(1));
        Assert.That(after.TileChunks.Values.Single().StateRevision, Is.EqualTo(1));
        Assert.That(edit.Changes.Single().Surfaces, Is.Empty);
        Assert.That(edit.Changes.Single().Regions, Is.Empty);
        Assert.That(edit.Changes.Single().TileChunks, Is.EqualTo(new[] { chunk.Key }));
        var context = new SpatialQueryContext(world, frame, "revision-check", uint.MaxValue,
            expectedGeometryRevisions: new Dictionary<EntityUid, ulong> { [frame] = 1 });
        Assert.That(entities.System<SpatialQuerySystem>().RayCast(context, Vector3.UnitZ * 3, -Vector3.UnitZ, 4), Is.Not.Null);
    }

    [Test]
    public void UnrepresentableDynamicGeometryRejectsPreparationWithoutChangingAnyFrame()
    {
        var (entities, geometry, physics, world) = Scene();
        var a = Frame(entities, world);
        var b = Frame(entities, world, Vector3.UnitX * 10, dynamic: true);
        var before = geometry.CaptureSnapshot(b);
        var source = before.Slabs.Values.Single();
        var tiny = new SlabSurface(source.Surface, source.Layer, new Box2(-1e-25f, -1e-25f, 1e-25f, 1e-25f), 0, 1e-25f);
        Assert.Throws<ArgumentException>(() => geometry.PrepareTransaction(new[]
        {
            Raised(geometry, a, 1), new SpatialGeometryEdit(before, slabs: new[] { tiny })
        }));
        Assert.That(geometry.CaptureSnapshot(a).Revision, Is.Zero);
        Assert.That(geometry.CaptureSnapshot(b).Revision, Is.Zero);
        Assert.That(physics.GetBodyCount(world), Is.EqualTo(2));
        var hit = physics.GetBackend(world).RayCast(Vector3.UnitZ * 3, -Vector3.UnitZ, 4, uint.MaxValue)!.Value;
        Assert.That(hit.Point.Z, Is.Zero.Within(1e-5));
        Assert.That(hit.Source.Revision, Is.Zero);
    }

    [Test]
    public void LayerChangesAdvanceAffectedSurfacesAndQueuedWorldTransferIsRejected()
    {
        var (entities, geometry, physics, world) = Scene();
        var frame = Frame(entities, world, tiles: true);
        var before = geometry.CaptureSnapshot(frame);
        using var layerEdit = geometry.PrepareTransaction(new[] { new SpatialGeometryEdit(before,
            layers: new Dictionary<LayerId, LayerDefinition> { [new LayerId(1)] = new(2, "Raised floor") }) });
        geometry.QueueTransaction(layerEdit);
        physics.Update(1f / 60);
        var after = geometry.CaptureSnapshot(frame);
        Assert.That(after.Slabs.Values.Single().StateRevision, Is.EqualTo(1));
        Assert.That(after.TileChunks.Values.Single().StateRevision, Is.EqualTo(1));
        Assert.That(after.SolidParts.Single().Bounds.Max.Z, Is.EqualTo(2));
        Assert.That(layerEdit.Changes.Single().Layers, Is.EqualTo(new[] { new LayerId(1) }));
        var hit = physics.GetBackend(world).RayCast(Vector3.UnitZ * 3, -Vector3.UnitZ, 4, uint.MaxValue)!.Value;
        Assert.That(hit.Point.Z, Is.EqualTo(2).Within(1e-5));
        using var prepared = geometry.PrepareTransaction(new[] { Empty(geometry, frame) });
        geometry.QueueTransaction(prepared);
        var destination = entities.System<SharedSpatialSystem>().CreateWorld();
        physics.EnablePhysics(destination);
        entities.System<SharedTransformSystem>().SetParent(frame, destination);
        physics.Update(1f / 60);
        Assert.That(prepared.Status, Is.EqualTo(WorldEditStatus.Rejected));
        Assert.That(geometry.CaptureSnapshot(frame).Revision, Is.EqualTo(1));
        Assert.That(physics.GetBodyCount(world), Is.Zero);
        Assert.That(physics.GetBodyCount(destination), Is.EqualTo(1));
    }

    [Test]
    public void EditQueuedByCompletionHandlerWaitsUntilTheFollowingBoundary()
    {
        var (entities, geometry, physics, world) = Scene();
        var frame = Frame(entities, world);
        using var first = geometry.PrepareTransaction(new[] { Raised(geometry, frame, 1) });
        WorldEditTransaction? second = null;
        entities.System<EditObserver>().Completed = tx =>
        {
            if (tx != first) return;
            second = geometry.PrepareTransaction(new[] { Raised(geometry, frame, 2) });
            geometry.QueueTransaction(second);
        };
        geometry.QueueTransaction(first);
        physics.Update(1f / 60);
        Assert.That(second!.Status, Is.EqualTo(WorldEditStatus.Queued));
        Assert.That(geometry.CaptureSnapshot(frame).Revision, Is.EqualTo(1));
        physics.Update(1f / 60);
        Assert.That(second.Status, Is.EqualTo(WorldEditStatus.Committed));
        Assert.That(geometry.CaptureSnapshot(frame).Revision, Is.EqualTo(2));
        second.Dispose();
    }

    [Test]
    public void DynamicGeometryCommitUsesCurrentCanonicalPoseAndVelocity()
    {
        var (entities, geometry, physics, world) = Scene();
        var frame = Frame(entities, world, dynamic: true);
        var body = entities.GetComponent<SpatialBodyComponent>(frame);
        using var edit = geometry.PrepareTransaction(new[] { Raised(geometry, frame, 1) });
        var pose = new RigidTransform3(new Vector3(2, 3, 4), Quaternion.CreateFromAxisAngle(Vector3.UnitZ, 0.4f));
        var linear = new Vector3(1, 2, 3);
        var angular = new Vector3(0.1f, 0.2f, 0.3f);
        entities.System<SharedTransformSystem>().SetWorldPose(frame, pose);
        physics.SetVelocity((frame, body), linear, angular);
        entities.System<EditObserver>().Completed = tx =>
        {
            Assert.That(tx.Status, Is.EqualTo(WorldEditStatus.Committed));
            Assert.That(entities.System<SharedTransformSystem>().GetWorldPose(frame), Is.EqualTo(pose));
            Assert.That(body.LinearVelocity, Is.EqualTo(linear));
            Assert.That(body.AngularVelocity, Is.EqualTo(angular));
            Assert.That(physics.GetBodyCount(world), Is.EqualTo(1));
        };
        geometry.QueueTransaction(edit);
        physics.Update(1f / 60);
        Assert.That(edit.Status, Is.EqualTo(WorldEditStatus.Committed));
        Assert.That(entities.System<SharedTransformSystem>().GetWorldPose(frame).Position.X, Is.GreaterThan(2));
    }

    [Reflect(false)]
    private sealed class EditObserver : EntitySystem
    {
        public int Count;
        public Action<WorldEditTransaction>? Completed;
        public override void Initialize() => SubscribeLocalEvent<WorldEditCompleted>(OnCompleted);
        private void OnCompleted(ref WorldEditCompleted ev)
        {
            Count++;
            Completed?.Invoke(ev.Transaction);
        }
    }
}
