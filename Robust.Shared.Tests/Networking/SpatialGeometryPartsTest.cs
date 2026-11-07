using System.IO;
using NUnit.Framework;
using Robust.Shared.GameObjects;
using Robust.Shared.Maths;
using Robust.Shared.Spatial;
using Robust.Shared.Spatial.Networking;

namespace Robust.Shared.Tests.Networking;

[TestFixture]
public sealed class SpatialGeometryPartsTest
{
    private static readonly EntityUid Frame = new(1);
    private static readonly LayerId Layer = new(1);
    private static readonly Dictionary<LayerId, LayerDefinition> Layers = new() { [Layer] = new(0, "Floor") };
    private static NetEntity Net(EntityUid uid) => new((int) uid);
    private static EntityUid Local(NetEntity uid) => new(uid.Id);

    [TestCase(16400, 0)]
    [TestCase(4500, 2000)]
    public void RecordAndByteOverflowSplitWithoutLosingSurfaces(int count, int materialLength)
    {
        var material = new string('m', materialLength);
        var original = new SpatialGeometrySnapshot(Frame, 0, Layers, Array.Empty<BoundarySegment>(),
            Enumerable.Range(0, count).Select(i => new SlabSurface(new SpatialSurfaceReference(Frame, new SurfaceId((uint) i)),
                Layer, new Box2(i, 0, i + 1, 1), 0, 0.2f, materialProfile: material)), Array.Empty<SpatialTileChunk>());
        Assert.Throws<InvalidDataException>(() => SpatialGeometryWireCodec.Encode(original, Net));
        var encoded = SpatialGeometryWireCodec.EncodeParts(original, Net);
        Assert.That(encoded.Count, Is.GreaterThan(1));
        Assert.That(encoded.All(p => p.Length <= SpatialGeometryWireCodec.MaximumBytes), Is.True);
        var merged = SpatialGeometryWireCodec.MergeParts(encoded.Select(p => SpatialGeometryWireCodec.Decode(p, Local)).ToArray());
        Assert.That(merged.Slabs.Count, Is.EqualTo(count));
        Assert.That(merged.SolidParts, Is.EqualTo(original.SolidParts));
        Assert.That(merged.Slabs.Values.All(s => s.MaterialProfile == material), Is.True);
    }

    [Test]
    public void LargeCoveringsRetainOneStructuralSlabAndEveryDecorativeCell()
    {
        var slab = new SlabSurface(new SpatialSurfaceReference(Frame, new SurfaceId(1)), Layer, new Box2(0, 0, 576, 16), 0, 0.2f);
        var tile = new SpatialFloorTile(new string('t', 1000), new SurfaceId(1));
        var chunks = Enumerable.Range(0, 36).Select(i => new SpatialTileChunk(new SpatialChunkKey(Layer, new Vector2i(i, 0)),
            Enumerable.Range(0, 256).Select(j => new KeyValuePair<Vector2i, SpatialFloorTile>(new Vector2i(j % 16, j / 16), tile))));
        var original = new SpatialGeometrySnapshot(Frame, 0, Layers, Array.Empty<BoundarySegment>(), new[] { slab }, chunks);
        Assert.Throws<InvalidDataException>(() => SpatialGeometryWireCodec.Encode(original, Net));
        var encoded = SpatialGeometryWireCodec.EncodeParts(original, Net);
        Assert.That(encoded.Count, Is.GreaterThan(1));
        var merged = SpatialGeometryWireCodec.MergeParts(encoded.Select(p => SpatialGeometryWireCodec.Decode(p, Local)).ToArray());
        Assert.That(merged.Slabs.Count, Is.EqualTo(1));
        Assert.That(merged.SolidParts.Count, Is.EqualTo(1));
        Assert.That(merged.TileChunks.Count, Is.EqualTo(36));
        foreach (var (key, chunk) in original.TileChunks)
            Assert.That(merged.TileChunks[key].Tiles, Is.EquivalentTo(chunk.Tiles));
    }

    [Test]
    public void CrossPartLayerSurfaceAndTileConflictsCannotPublish()
    {
        var slab = new SlabSurface(new SpatialSurfaceReference(Frame, new SurfaceId(1)), Layer, new Box2(0, 0, 1, 1), 0, 0.2f);
        var a = new SpatialGeometrySnapshot(Frame, 0, Layers, Array.Empty<BoundarySegment>(), new[] { slab }, Array.Empty<SpatialTileChunk>());
        var differentLayer = new SpatialGeometrySnapshot(Frame, 0, new Dictionary<LayerId, LayerDefinition> { [Layer] = new(1, "Floor") },
            Array.Empty<BoundarySegment>(), new[] { slab }, Array.Empty<SpatialTileChunk>());
        Assert.Throws<InvalidDataException>(() => SpatialGeometryWireCodec.MergeParts(new[] { a, differentLayer }));
        var differentSlab = new SpatialGeometrySnapshot(Frame, 0, Layers, Array.Empty<BoundarySegment>(),
            new[] { new SlabSurface(slab.Surface, Layer, slab.Footprint, 1, 0.2f) }, Array.Empty<SpatialTileChunk>());
        Assert.Throws<InvalidDataException>(() => SpatialGeometryWireCodec.MergeParts(new[] { a, differentSlab }));
        var withTile = new SpatialGeometrySnapshot(Frame, 0, Layers, Array.Empty<BoundarySegment>(), new[] { slab },
            new[] { new SpatialTileChunk(new SpatialChunkKey(Layer, Vector2i.Zero),
                new Dictionary<Vector2i, SpatialFloorTile> { [Vector2i.Zero] = new("metal", new SurfaceId(1)) }) });
        Assert.Throws<InvalidDataException>(() => SpatialGeometryWireCodec.MergeParts(new[] { withTile, withTile }));
    }
}
