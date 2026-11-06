using NUnit.Framework;
using Robust.Shared.GameObjects;
using Robust.Shared.Map.Components;
using Robust.Shared.Maths;
using Robust.Shared.Spatial;
using Robust.UnitTesting.Server;

namespace Robust.UnitTesting.Shared.GameObjects.Systems;

[TestFixture]
public sealed class SpatialGeometryTests
{
    [Test]
    public void CanonicalInitializationRejectsDuplicateEdgesAndSurfaceIdsAtomically()
    {
        var sim = RobustServerSimulation.NewSimulation().InitializeInstance();
        var entities = sim.Resolve<IEntityManager>();
        var spatial = entities.System<SharedSpatialSystem>();
        var geometry = entities.System<SharedGeometrySystem>();
        var world = spatial.CreateWorld();
        var frame = CreateUninitializedFrame(entities, world);
        var component = entities.GetComponent<SpatialGeometryComponent>(frame);
        var layer = new LayerId(5);
        var east = BoundaryKey.FromCell(frame, layer, new Vector2i(-2, 0), CellBoundary.East);
        var west = BoundaryKey.FromCell(frame, layer, new Vector2i(-1, 0), CellBoundary.West);
        var boundary = new BoundarySegment(east, new SurfaceId(10), 0.2f, 0, 3, stateRevision: 7);
        geometry.ConfigureInitialGeometry((frame, component), new[] { boundary }, Array.Empty<SlabSurface>(), 7);
        var before = geometry.CaptureSnapshot(frame);
        var duplicate = new BoundarySegment(west, new SurfaceId(11), 0.2f, 0, 3, stateRevision: 7);
        Assert.Throws<ArgumentException>(() => geometry.ConfigureInitialGeometry((frame, component),
            new[] { boundary, duplicate }, Array.Empty<SlabSurface>(), 7));
        var duplicateId = new SlabSurface(new SpatialSurfaceReference(frame, new SurfaceId(10)), layer,
            new Box2(-2, -2, 2, 2), 0, 0.2f);
        Assert.Throws<ArgumentException>(() => geometry.ConfigureInitialGeometry((frame, component),
            new[] { boundary }, new[] { duplicateId }, 7));
        Assert.Throws<ArgumentException>(() => geometry.ConfigureInitialGeometry((frame, component),
            new[] { boundary }, Array.Empty<SlabSurface>(), 6));
        var foreign = new BoundarySegment(new BoundaryKey(world, layer, 0, 0, BoundaryAxis.Horizontal),
            new SurfaceId(12), 0.2f, 0, 3);
        Assert.Throws<ArgumentException>(() => geometry.ConfigureInitialGeometry((frame, component),
            new[] { foreign }, Array.Empty<SlabSurface>()));
        var missingLayer = new BoundarySegment(new BoundaryKey(frame, new LayerId(999), 0, 0, BoundaryAxis.Horizontal),
            new SurfaceId(12), 0.2f, 0, 3);
        Assert.Throws<KeyNotFoundException>(() => geometry.ConfigureInitialGeometry((frame, component),
            new[] { missingLayer }, Array.Empty<SlabSurface>()));
        var after = geometry.CaptureSnapshot(frame);
        Assert.That(after.Revision, Is.EqualTo(before.Revision));
        Assert.That(after.Boundaries.Keys, Is.EqualTo(before.Boundaries.Keys));
        Assert.That(after.SolidParts, Is.EqualTo(before.SolidParts));
        entities.DeleteEntity(frame);
    }

    [Test]
    public void CapturesRemainIndependentAndLiveEditsCannotUseInitializationApi()
    {
        var sim = RobustServerSimulation.NewSimulation().InitializeInstance();
        var entities = sim.Resolve<IEntityManager>();
        var spatial = entities.System<SharedSpatialSystem>();
        var geometry = entities.System<SharedGeometrySystem>();
        var world = spatial.CreateWorld();
        var frame = CreateUninitializedFrame(entities, world);
        var component = entities.GetComponent<SpatialGeometryComponent>(frame);
        var surface = new SpatialSurfaceReference(frame, new SurfaceId(20));
        var slab = new SlabSurface(surface, new LayerId(5), new Box2(-2, -2, 2, 2), 0, 0.2f);
        var input = new List<SlabSurface> { slab };
        geometry.ConfigureInitialGeometry((frame, component), Array.Empty<BoundarySegment>(), input);
        input.Clear();
        var old = geometry.CaptureSnapshot(frame);
        Assert.That(old.Slabs, Has.Count.EqualTo(1));
        Assert.That(old.Slabs[surface.Surface], Is.Not.SameAs(slab));
        var parts = old.GetSurfaceParts(surface.Surface);
        Assert.That(parts, Has.Count.EqualTo(1));
        Assert.Throws<InvalidOperationException>(() => geometry.GetSurfaceParts(old,
            new SpatialSurfaceReference(world, surface.Surface)));

        geometry.ConfigureInitialGeometry((frame, component), Array.Empty<BoundarySegment>(), Array.Empty<SlabSurface>(), 1);
        var next = geometry.CaptureSnapshot(frame);
        Assert.That(next.SolidParts, Is.Empty);
        Assert.That(next.Revision, Is.EqualTo(1));
        Assert.That(old.SolidParts, Has.Count.EqualTo(1));
        Assert.That(old.Revision, Is.Zero);
        entities.InitializeAndStartEntity(frame, entities.GetComponent<MapComponent>(world).MapId);
        Assert.Throws<InvalidOperationException>(() => geometry.ConfigureInitialGeometry((frame, component),
            Array.Empty<BoundarySegment>(), new[] { slab }, 2));
        entities.DeleteEntity(frame);
        Assert.Throws<InvalidOperationException>(() => geometry.CaptureSnapshot(frame));
        // Detached data remains readable for a worker after deletion; installation
        // must separately validate that the owning frame/revision is still current.
        Assert.That(old.GetSurfaceParts(surface.Surface), Is.EqualTo(parts));
    }

    internal static EntityUid CreateUninitializedFrame(IEntityManager entities, EntityUid world)
    {
        var frame = entities.CreateEntityUninitialized(null, world, RigidTransform3.Identity);
        var layers = entities.AddComponent<SpatialFrameComponent>(frame);
        entities.System<SharedSpatialSystem>().ConfigureInitialLayers((frame, layers),
            new Dictionary<LayerId, LayerDefinition> { [new LayerId(5)] = new(1.75f, "Mezzanine", 1) });
        entities.AddComponent<SpatialGeometryComponent>(frame);
        return frame;
    }

    [Test]
    public void SparseFloorCoveringsResolveSlabsWithoutCreatingOrRemovingCollision()
    {
        var sim = RobustServerSimulation.NewSimulation().InitializeInstance();
        var entities = sim.Resolve<IEntityManager>();
        var world = entities.System<SharedSpatialSystem>().CreateWorld();
        var geometry = entities.System<SharedGeometrySystem>();
        var frame = CreateUninitializedFrame(entities, world);
        var component = entities.GetComponent<SpatialGeometryComponent>(frame);
        var lower = new LayerId(5);
        var upper = new LayerId(999);
        entities.System<SharedSpatialSystem>().ConfigureInitialLayers(
            (frame, entities.GetComponent<SpatialFrameComponent>(frame)),
            new Dictionary<LayerId, LayerDefinition>
            {
                [lower] = new(0, "Lower"), [upper] = new(38.25f, "Upper")
            });
        var cells = new[] { new Vector2i(-1000000, -1), new Vector2i(1000000, 17) };
        var slabs = new List<SlabSurface>();
        var chunks = new List<SpatialTileChunk>();
        uint nextId = 0;
        foreach (var layer in new[] { lower, upper })
        foreach (var cell in cells)
        {
            var id = new SurfaceId(nextId++);
            slabs.Add(new SlabSurface(new SpatialSurfaceReference(frame, id), layer,
                new Box2(cell.X, cell.Y, cell.X + 1, cell.Y + 1), 0, 0.25f));
            chunks.Add(new SpatialTileChunk(SpatialChunkKey.FromCell(layer, cell),
                new Dictionary<Vector2i, SpatialFloorTile>
                {
                    [SpatialChunkKey.LocalCell(cell)] = new("covering", id)
                }));
        }
        geometry.ConfigureInitialGeometry((frame, component), Array.Empty<BoundarySegment>(), slabs, tileChunks: chunks);
        var before = geometry.CaptureSnapshot(frame);
        Assert.That(before.TileChunks, Has.Count.EqualTo(4));
        Assert.That(before.TileChunks.Values.Sum(c => c.Tiles.Count), Is.EqualTo(4));
        Assert.That(before.TryGetFloorTile(lower, cells[0], out var a), Is.True);
        Assert.That(before.TryGetFloorTile(upper, cells[0], out var b), Is.True);
        Assert.That(a.StructuralSurface, Is.Not.EqualTo(b.StructuralSurface));
        Assert.That(before.TryGetFloorTile(lower, Vector2i.Zero, out _), Is.False);

        var wrongLayer = new SpatialTileChunk(SpatialChunkKey.FromCell(upper, cells[0]), chunks[0].Tiles);
        Assert.Throws<ArgumentException>(() => geometry.ConfigureInitialGeometry((frame, component),
            Array.Empty<BoundarySegment>(), slabs, tileChunks: new[] { wrongLayer }));
        Assert.Throws<ArgumentException>(() => geometry.ConfigureInitialGeometry((frame, component),
            Array.Empty<BoundarySegment>(), slabs, tileChunks: new[] { chunks[0], chunks[0] }));
        Assert.Throws<ArgumentException>(() => geometry.ConfigureInitialGeometry((frame, component),
            Array.Empty<BoundarySegment>(), Array.Empty<SlabSurface>(), tileChunks: chunks));
        Assert.That(geometry.CaptureSnapshot(frame).TileChunks.Keys, Is.EqualTo(before.TileChunks.Keys));

        geometry.ConfigureInitialGeometry((frame, component), Array.Empty<BoundarySegment>(), slabs, 1);
        var withoutCoverings = geometry.CaptureSnapshot(frame);
        Assert.That(withoutCoverings.TileChunks, Is.Empty);
        Assert.That(withoutCoverings.SolidParts, Is.EqualTo(before.SolidParts));
        Assert.That(before.TileChunks, Has.Count.EqualTo(4));
        entities.DeleteEntity(frame);
    }
}
