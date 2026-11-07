using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using Robust.Shared.GameObjects;

namespace Robust.Shared.Spatial.Networking;

internal static partial class SpatialGeometryWireCodec
{
    private sealed class PayloadBudgetExceeded(string message) : Exception(message);

    private static void EncodingPartitionBudget(int holes, ref long work)
    {
        work += (2L * holes + 1) * (holes + 1);
        if (holes < 0 || work > MaximumPartitionWork)
            throw new PayloadBudgetExceeded("Geometry partition work budget exceeded.");
    }

    /// <summary>
    /// Partition one authorized frame without changing its semantic identities or
    /// revision. Each part is independently bounded and decodable. A slab whose
    /// decorative chunks span multiple parts is repeated as an exact dependency;
    /// it is merged once before publishing the frame.
    /// </summary>
    internal static IReadOnlyList<byte[]> EncodeParts(SpatialGeometrySnapshot snapshot, Func<EntityUid, NetEntity> getNetEntity)
    {
        var parts = new List<byte[]>();
        long bytes = 0;
        Append(snapshot);
        return parts;

        void Append(SpatialGeometrySnapshot selected)
        {
            try
            {
                var encoded = EncodeCore(selected, getNetEntity);
                bytes += encoded.Length;
                if (parts.Count == SpatialGeometryManifest.MaximumParts || bytes > SpatialGeometryManifest.MaximumBytes)
                    throw new InvalidDataException("Geometry selection exceeds the complete transfer budget.");
                parts.Add(encoded);
                return;
            }
            catch (PayloadBudgetExceeded)
            {
                var surfaces = selected.Boundaries.Values.Select(b => b.Surface).Concat(selected.Slabs.Keys)
                    .OrderBy(s => s.Value).ToArray();
                if (surfaces.Length > 1)
                {
                    var half = surfaces.Length / 2;
                    Append(Select(selected, surfaces.Take(half).ToHashSet()));
                    Append(Select(selected, surfaces.Skip(half).ToHashSet()));
                    return;
                }
                var tiles = selected.TileChunks.Values.OrderBy(c => c.Key.Layer.Value)
                    .ThenBy(c => c.Key.Indices.X).ThenBy(c => c.Key.Indices.Y).ToArray();
                if (tiles.Length > 1)
                {
                    var half = tiles.Length / 2;
                    Append(WithTiles(tiles.Take(half)));
                    Append(WithTiles(tiles.Skip(half)));
                    return;
                }
                throw new InvalidDataException("A single geometry surface exceeds the payload budget.");

                SpatialGeometrySnapshot WithTiles(IEnumerable<SpatialTileChunk> chunks)
                    => new(selected.OwnerFrame, selected.Revision, selected.Layers,
                        selected.Boundaries.Values, selected.Slabs.Values, chunks);
            }
        }
    }

    /// <summary>Validate cross-part dependencies before atomically publishing the complete frame.</summary>
    internal static SpatialGeometrySnapshot MergeParts(IReadOnlyList<SpatialGeometrySnapshot> parts)
    {
        if (parts.Count == 0) throw new InvalidDataException("A geometry frame requires a payload.");
        if (parts.Count == 1) return parts[0];
        var first = parts[0];
        var layers = new Dictionary<LayerId, LayerDefinition>();
        var boundaries = new Dictionary<BoundaryKey, BoundarySegment>();
        var slabs = new Dictionary<SurfaceId, SlabSurface>();
        var tiles = new Dictionary<SpatialChunkKey, SpatialTileChunk>();
        foreach (var part in parts)
        {
            if (part.OwnerFrame != first.OwnerFrame || part.Revision != first.Revision)
                throw new InvalidDataException("Geometry parts disagree on their owner or revision.");
            foreach (var (id, layer) in part.Layers)
            {
                if (layers.TryGetValue(id, out var old) && old != layer)
                    throw new InvalidDataException("Geometry parts disagree on a layer.");
                layers[id] = layer;
            }
            foreach (var (key, boundary) in part.Boundaries)
                if (!boundaries.TryAdd(key, boundary)) throw new InvalidDataException("Duplicate boundary across geometry parts.");
            foreach (var (id, slab) in part.Slabs)
            {
                if (slabs.TryGetValue(id, out var old))
                {
                    if (old.Surface != slab.Surface || old.Layer != slab.Layer || old.Footprint != slab.Footprint
                        || old.TopElevation != slab.TopElevation || old.Thickness != slab.Thickness || old.Entity != slab.Entity
                        || old.MaterialProfile != slab.MaterialProfile || old.StateRevision != slab.StateRevision
                        || !old.Openings.SequenceEqual(slab.Openings))
                        throw new InvalidDataException("Geometry parts disagree on a structural slab.");
                }
                else slabs.Add(id, slab);
            }
            foreach (var (key, chunk) in part.TileChunks)
            {
                if (!tiles.TryGetValue(key, out var old)) { tiles.Add(key, chunk); continue; }
                if (old.StateRevision != chunk.StateRevision || old.Tiles.Keys.Intersect(chunk.Tiles.Keys).Any())
                    throw new InvalidDataException("Conflicting decorative tiles across geometry parts.");
                tiles[key] = new SpatialTileChunk(key, old.Tiles.Concat(chunk.Tiles), old.StateRevision);
            }
        }
        return new SpatialGeometrySnapshot(first.OwnerFrame, first.Revision, layers, boundaries.Values, slabs.Values, tiles.Values);
    }
}
