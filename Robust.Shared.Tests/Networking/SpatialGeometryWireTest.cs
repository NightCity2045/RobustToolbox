using System.Buffers.Binary;
using NUnit.Framework;
using Robust.Shared.GameObjects;
using Robust.Shared.Maths;
using Robust.Shared.Spatial;
using Robust.Shared.Spatial.Networking;

namespace Robust.Shared.Tests.Networking;

[TestFixture]
public sealed class SpatialGeometryWireTest
{
    private static SpatialGeometrySnapshot Scene()
    {
        var frame = new EntityUid(1);
        var layer = new LayerId(7);
        var floor = new SurfaceId(12);
        var hidden = new SurfaceId(13);
        return new SpatialGeometrySnapshot(frame, 9,
            new Dictionary<LayerId, LayerDefinition> { [layer] = new(4.25f, "Уровень −1") },
            new[] { new BoundarySegment(new BoundaryKey(frame, layer, -3, 0, BoundaryAxis.Horizontal),
                new SurfaceId(10), 0.2f, 0, 3, new[] { new BoundaryOpening(0.2f, 0.8f, 0, 2) },
                "wall", new EntityUid(2), 5) },
            new[]
            {
                new SlabSurface(new SpatialSurfaceReference(frame, floor), layer, new Box2(-16, 0, -14, 2),
                    0, 0.3f, materialProfile: "steel", entity: new EntityUid(3), stateRevision: 7),
                new SlabSurface(new SpatialSurfaceReference(frame, hidden), layer, new Box2(-16, 0, -14, 2),
                    0, 0.3f, materialProfile: "hidden-material", entity: new EntityUid(4), stateRevision: 8)
            },
            new[] { new SpatialTileChunk(new SpatialChunkKey(layer, new Vector2i(-1, 0)),
                new Dictionary<Vector2i, SpatialFloorTile>
                {
                    [new Vector2i(0, 0)] = new("paint", floor),
                    [new Vector2i(1, 0)] = new("hidden-covering", hidden)
                }, 9) });
    }

    [Test]
    public void SemanticGeometryRoundTripsWithNetworkReferenceRemapping()
    {
        var before = Scene();
        var bytes = SpatialGeometryWireCodec.Encode(before, uid => new NetEntity((int) uid + 1000));
        var decoded = SpatialGeometryWireCodec.Decode(bytes, net => new EntityUid(net.Id + 2000));
        Assert.That(decoded.OwnerFrame, Is.EqualTo(new EntityUid(3001)));
        Assert.That(decoded.Revision, Is.EqualTo(before.Revision));
        Assert.That(decoded.Layers, Is.EqualTo(before.Layers));
        Assert.That(decoded.SolidParts.Select(s => (s.Bounds, s.MaterialProfile, s.Revision)),
            Is.EqualTo(before.SolidParts.Select(s => (s.Bounds, s.MaterialProfile, s.Revision))));
        Assert.That(decoded.Boundaries.Values.Single().Key.OwnerFrame, Is.EqualTo(decoded.OwnerFrame));
        Assert.That(decoded.Boundaries.Values.Single().Entity, Is.EqualTo(new EntityUid(3002)));
        Assert.That(decoded.TileChunks.Values.Single().Tiles, Is.EqualTo(before.TileChunks.Values.Single().Tiles));
        Assert.That(SpatialGeometryWireCodec.Encode(decoded, uid => new NetEntity((int) uid - 2000)), Is.EqualTo(bytes));
        Assert.That(decoded.CaptureIdentity, Is.Null);
        Assert.Throws<ArgumentException>(() => new SpatialGeometryEdit(decoded));
    }

    [Test]
    public void SelectionExcludesUnauthorizedSurfacesReferencesAndCoverings()
    {
        var selected = SpatialGeometryWireCodec.Select(Scene(), new HashSet<SurfaceId> { new(12) });
        var encodedReferences = new List<EntityUid>();
        var bytes = SpatialGeometryWireCodec.Encode(selected, uid =>
        {
            encodedReferences.Add(uid);
            return new NetEntity((int) uid);
        });
        Assert.That(encodedReferences, Is.EqualTo(new[] { new EntityUid(1), new EntityUid(3) }));
        var decoded = SpatialGeometryWireCodec.Decode(bytes, net => new EntityUid(net.Id));
        Assert.That(decoded.Boundaries, Is.Empty);
        Assert.That(decoded.Slabs.Keys, Is.EqualTo(new[] { new SurfaceId(12) }));
        Assert.That(decoded.TileChunks.Values.Single().Tiles.Values.Select(t => t.MaterialProfile), Is.EqualTo(new[] { "paint" }));
    }

    [Test]
    public void InvalidCountsReferencesVersionsAndTrailingDataAreRejected()
    {
        var bytes = SpatialGeometryWireCodec.Encode(Scene(), uid => new NetEntity((int) uid));
        void Reject(byte[] changed) => Assert.Throws<System.IO.InvalidDataException>(() =>
            SpatialGeometryWireCodec.Decode(changed, net => new EntityUid(net.Id)));
        foreach (var count in new[] { -1, 0, int.MaxValue })
        {
            var changed = bytes.ToArray();
            BinaryPrimitives.WriteInt32LittleEndian(changed.AsSpan(8), count);
            Reject(changed);
        }
        foreach (var id in new[] { -1, 0, NetEntity.ClientEntity })
        {
            var changed = bytes.ToArray();
            BinaryPrimitives.WriteInt32LittleEndian(changed.AsSpan(12), id);
            Reject(changed);
        }
        var duplicate = bytes.ToArray();
        duplicate.AsSpan(12, 4).CopyTo(duplicate.AsSpan(16, 4));
        Reject(duplicate);
        var version = bytes.ToArray();
        version[4] = 255;
        Reject(version);
        Reject(bytes.Concat(new byte[] { 0 }).ToArray());
        for (var length = 0; length < bytes.Length; length++)
            Assert.Catch(() => SpatialGeometryWireCodec.Decode(bytes[..length], net => new EntityUid(net.Id)), $"Truncation at {length}");
        Assert.Throws<System.IO.InvalidDataException>(() =>
            SpatialGeometryWireCodec.Encode(Scene(), _ => new NetEntity(1)), "Aliased network mappings are not valid geometry.");
    }

    [Test]
    public void TextAndPartitionBudgetsRejectHostilePayloadsBeforeExpansion()
    {
        var empty = new SpatialGeometrySnapshot(new EntityUid(1), 0,
            new Dictionary<LayerId, LayerDefinition> { [new LayerId(1)] = new(0, "x") },
            Array.Empty<BoundarySegment>(), Array.Empty<SlabSurface>(), Array.Empty<SpatialTileChunk>());
        var bytes = SpatialGeometryWireCodec.Encode(empty, uid => new NetEntity((int) uid));
        var invalidUtf8 = bytes.ToArray();
        invalidUtf8[44] = 0xff;
        Assert.Throws<System.Text.DecoderFallbackException>(() =>
            SpatialGeometryWireCodec.Decode(invalidUtf8, net => new EntityUid(net.Id)));
        var oversizedText = bytes.ToArray();
        BinaryPrimitives.WriteInt32LittleEndian(oversizedText.AsSpan(40), 4097);
        Assert.Throws<System.IO.InvalidDataException>(() =>
            SpatialGeometryWireCodec.Decode(oversizedText, net => new EntityUid(net.Id)));

        using var stream = new System.IO.MemoryStream();
        using var writer = new System.IO.BinaryWriter(stream);
        // Keep the valid prefix through the layer and empty boundary collection.
        writer.Write(bytes.AsSpan(0, 49));
        writer.Write(1); // One slab.
        writer.Write(1u); writer.Write(1u); // Surface and layer.
        writer.Write(0f); writer.Write(0f); writer.Write(1f); writer.Write(1f);
        writer.Write(0f); writer.Write(0.2f); writer.Write(-1); writer.Write(0ul);
        writer.Write(0); // Empty material string.
        writer.Write(1500);
        writer.Write(new byte[1500 * 16]);
        writer.Write(0); // Empty tile chunks.
        Assert.Throws<System.IO.InvalidDataException>(() =>
            SpatialGeometryWireCodec.Decode(stream.ToArray(), net => new EntityUid(net.Id)),
            "Opening count must be bounded before constructing a quadratic partition.");
    }

    [Test]
    public void MissingReferenceDefersBeforeGeometryConstruction()
    {
        var bytes = SpatialGeometryWireCodec.Encode(Scene(), uid => new NetEntity((int) uid));
        var attempted = new List<NetEntity>();
        Assert.Throws<KeyNotFoundException>(() => SpatialGeometryWireCodec.Decode(bytes, net =>
        {
            attempted.Add(net);
            if (net.Id == 2) throw new KeyNotFoundException("Parent/source has not arrived.");
            return new EntityUid(net.Id);
        }));
        Assert.That(attempted, Is.EqualTo(new[] { new NetEntity(1), new NetEntity(2) }));
        Assert.That(SpatialGeometryWireCodec.Decode(bytes, net => new EntityUid(net.Id)).SolidParts, Is.Not.Empty);
    }
}
