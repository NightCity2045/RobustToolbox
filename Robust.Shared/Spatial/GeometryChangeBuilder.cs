using System.Collections.Generic;
using System.Linq;

namespace Robust.Shared.Spatial;

/// <summary>Revisions are assigned by the authority, never trusted from an edit's input objects.</summary>
internal static class GeometryChangeBuilder
{
    public static SpatialGeometryChange Build(SpatialGeometrySnapshot before, SpatialGeometrySnapshot draft)
    {
        var layers = before.Layers.Keys.Union(draft.Layers.Keys)
            .Where(key => !before.Layers.TryGetValue(key, out var a) || !draft.Layers.TryGetValue(key, out var b) || a != b).ToHashSet();
        var surfaces = new HashSet<SurfaceId>();
        var boundaries = new List<BoundarySegment>();
        foreach (var value in draft.Boundaries.Values)
        {
            var unchanged = before.Boundaries.TryGetValue(value.Key, out var old) && BoundaryEqual(old, value)
                && !layers.Contains(value.Key.LayerId);
            if (!unchanged) surfaces.Add(value.Surface);
            boundaries.Add(new BoundarySegment(value.Key, value.Surface, value.Thickness, value.BaseElevation, value.Height,
                value.Openings, value.MaterialProfile, value.Entity, unchanged ? old!.StateRevision : draft.Revision));
        }
        foreach (var old in before.Boundaries.Values)
            if (!draft.Boundaries.TryGetValue(old.Key, out var next) || !BoundaryEqual(old, next) || layers.Contains(old.Key.LayerId))
                surfaces.Add(old.Surface);

        var slabs = new List<SlabSurface>();
        foreach (var value in draft.Slabs.Values)
        {
            var unchanged = before.Slabs.TryGetValue(value.Surface.Surface, out var old) && SlabEqual(old, value)
                && !layers.Contains(value.Layer);
            if (!unchanged) surfaces.Add(value.Surface.Surface);
            slabs.Add(new SlabSurface(value.Surface, value.Layer, value.Footprint, value.TopElevation, value.Thickness,
                value.Openings, value.MaterialProfile, value.Entity, unchanged ? old!.StateRevision : draft.Revision));
        }
        foreach (var old in before.Slabs.Values)
            if (!draft.Slabs.TryGetValue(old.Surface.Surface, out var next) || !SlabEqual(old, next) || layers.Contains(old.Layer))
                surfaces.Add(old.Surface.Surface);

        var changedChunks = new HashSet<SpatialChunkKey>();
        var chunks = new List<SpatialTileChunk>();
        foreach (var value in draft.TileChunks.Values)
        {
            var unchanged = before.TileChunks.TryGetValue(value.Key, out var old) && ChunkEqual(old, value)
                && !layers.Contains(value.Key.Layer);
            if (!unchanged) changedChunks.Add(value.Key);
            chunks.Add(new SpatialTileChunk(value.Key, value.Tiles, unchanged ? old!.StateRevision : draft.Revision));
        }
        foreach (var old in before.TileChunks.Values)
            if (!draft.TileChunks.ContainsKey(old.Key)) changedChunks.Add(old.Key);
        var after = new SpatialGeometrySnapshot(draft.OwnerFrame, draft.Revision, draft.Layers, boundaries, slabs, chunks, before.CaptureIdentity);
        return new SpatialGeometryChange(before, after, surfaces, changedChunks, layers);
    }

    private static bool BoundaryEqual(BoundarySegment a, BoundarySegment b)
        => a.Key == b.Key && a.Surface == b.Surface && a.Entity == b.Entity && a.Thickness == b.Thickness
            && a.BaseElevation == b.BaseElevation && a.Height == b.Height && a.MaterialProfile == b.MaterialProfile
            && a.Openings.SequenceEqual(b.Openings);

    private static bool SlabEqual(SlabSurface a, SlabSurface b)
        => a.Surface == b.Surface && a.Layer == b.Layer && a.Footprint == b.Footprint && a.TopElevation == b.TopElevation
            && a.Thickness == b.Thickness && a.Entity == b.Entity && a.MaterialProfile == b.MaterialProfile
            && a.Openings.SequenceEqual(b.Openings);

    private static bool ChunkEqual(SpatialTileChunk a, SpatialTileChunk b)
        => a.Key == b.Key && a.Tiles.Count == b.Tiles.Count
            && a.Tiles.All(pair => b.Tiles.TryGetValue(pair.Key, out var other) && pair.Value == other);
}
