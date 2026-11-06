using NUnit.Framework;
using Robust.Shared.ContentPack;
using Robust.Shared.EntitySerialization;
using Robust.Shared.EntitySerialization.Systems;
using Robust.Shared.GameObjects;
using Robust.Shared.Map.Components;
using Robust.Shared.Maths;
using Robust.Shared.Spatial;
using Robust.Shared.Utility;
using Robust.UnitTesting.Shared.GameObjects.Systems;

namespace Robust.UnitTesting.Shared.EntitySerialization;

[TestFixture]
public sealed class SpatialGeometryPersistenceTest : RobustIntegrationTest
{
    [Test]
    public async Task CanonicalGeometryRestoresWithoutSerializingDerivedShapes()
    {
        using var server = StartServer(new ServerIntegrationOptions { Pool = false });
        await server.WaitIdleAsync();
        await server.WaitAssertion(() =>
        {
            var entities = server.EntMan;
            var geometry = server.System<SharedGeometrySystem>();
            var world = server.System<SharedSpatialSystem>().CreateWorld();
            var frame = SpatialGeometryTests.CreateUninitializedFrame(entities, world);
            var layer = new LayerId(5);
            var boundary = new BoundarySegment(new BoundaryKey(frame, layer, -2, 0, BoundaryAxis.Horizontal),
                new SurfaceId(3), 0.15f, 0, 3, new[] { new BoundaryOpening(0.25f, 0.75f, 1, 2) },
                "wall-profile", frame, 12);
            var slab = new SlabSurface(new SpatialSurfaceReference(frame, new SurfaceId(4)), layer,
                new Box2(-2, -2, 2, 2), 0, 0.2f, new[] { new Box2(-0.5f, -0.5f, 0.5f, 0.5f) }, stateRevision: 10);
            var component = entities.GetComponent<SpatialGeometryComponent>(frame);
            var cell = new Vector2i(-1, -1);
            var tile = new SpatialFloorTile("decorative-floor", slab.Surface.Surface);
            var chunk = new SpatialTileChunk(SpatialChunkKey.FromCell(layer, cell),
                new Dictionary<Vector2i, SpatialFloorTile> { [SpatialChunkKey.LocalCell(cell)] = tile }, 11);
            geometry.ConfigureInitialGeometry((frame, component), new[] { boundary }, new[] { slab }, 12, new[] { chunk });
            entities.InitializeAndStartEntity(frame, entities.GetComponent<MapComponent>(world).MapId);
            var original = geometry.CaptureSnapshot(frame);
            var loader = server.System<MapLoaderSystem>();
            var path = new ResPath("/canonical-spatial-geometry.yml");
            Assert.That(loader.TrySaveMap(entities.GetComponent<MapComponent>(world).MapId, path), Is.True);
            using (var reader = server.Resolve<IResourceManager>().UserData.OpenText(path))
            {
                var saved = reader.ReadToEnd();
                Assert.That(saved, Does.Contain("type: SpatialGeometry"));
                Assert.That(saved, Does.Contain("tileChunks:"));
                Assert.That(saved, Does.Not.Contain("solidParts:"));
                Assert.That(saved, Does.Not.Contain("shapeIndex:"));
            }
            entities.DeleteEntity(world);
            Assert.That(loader.TryLoadMap(path, out var restored, out _), Is.True);
            Assert.That(loader.TryLoadMap(path, out var clone, out _,
                new DeserializationOptions { RegenerateSpatialWorldIds = true }), Is.True);
            var surfaces = new HashSet<SpatialSurfaceReference>();
            foreach (var map in new[] { restored!.Value.Owner, clone!.Value.Owner })
            {
                using var children = entities.GetComponent<TransformComponent>(map).ChildEnumerator;
                Assert.That(children.MoveNext(out var loadedFrame), Is.True);
                Assert.That(children.MoveNext(out _), Is.False);
                var snapshot = geometry.CaptureSnapshot(loadedFrame);
                Assert.That(snapshot.Revision, Is.EqualTo(12));
                Assert.That(snapshot.TileChunks, Has.Count.EqualTo(1));
                Assert.That(snapshot.TileChunks[chunk.Key].StateRevision, Is.EqualTo(11));
                Assert.That(snapshot.TryGetFloorTile(layer, cell, out var loadedTile), Is.True);
                Assert.That(loadedTile, Is.EqualTo(tile));
                Assert.That(snapshot.Slabs[loadedTile.StructuralSurface].Surface.OwnerFrame, Is.EqualTo(loadedFrame));
                Assert.That(snapshot.SolidParts.Select(p => p.Bounds), Is.EqualTo(original.SolidParts.Select(p => p.Bounds)));
                Assert.That(snapshot.GetSurfaceParts(new SurfaceId(3)).All(p => p.Entity == loadedFrame), Is.True);
                Assert.That(snapshot.GetSurfaceParts(new SurfaceId(4)).All(p => p.Entity == null), Is.True);
                Assert.That(snapshot.SolidParts.All(p => p.Surface.OwnerFrame == loadedFrame), Is.True);
                surfaces.Add(new SpatialSurfaceReference(loadedFrame, new SurfaceId(3)));
            }
            Assert.That(surfaces, Has.Count.EqualTo(2));
        });
    }
}
