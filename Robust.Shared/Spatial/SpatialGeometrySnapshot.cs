using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using Robust.Shared.GameObjects;
using Robust.Shared.Maths;

namespace Robust.Shared.Spatial;

/// <summary>
/// Validated detached snapshot for geometry consumers and worker jobs. Capturing
/// copies semantic definitions; later component deserialization cannot mutate a
/// worker's snapshot. Revision acceptance and installation belong to transactions.
/// </summary>
public sealed class SpatialGeometrySnapshot
{
    internal object? CaptureIdentity { get; }
    public EntityUid OwnerFrame { get; }
    public ulong Revision { get; }
    public IReadOnlyDictionary<LayerId, LayerDefinition> Layers { get; }
    public IReadOnlyDictionary<BoundaryKey, BoundarySegment> Boundaries { get; }
    public IReadOnlyDictionary<SurfaceId, SlabSurface> Slabs { get; }
    public IReadOnlyDictionary<SpatialChunkKey, SpatialTileChunk> TileChunks { get; }
    public IReadOnlyList<SpatialSolidPart> SolidParts { get; }
    private readonly Dictionary<SurfaceId, IReadOnlyList<SpatialSolidPart>> _surfaces = new();

    internal SpatialGeometrySnapshot(EntityUid frame, ulong revision,
        IReadOnlyDictionary<LayerId, LayerDefinition> layers,
        IEnumerable<BoundarySegment> boundaries, IEnumerable<SlabSurface> slabs,
        IEnumerable<SpatialTileChunk> tileChunks, object? captureIdentity = null)
    {
        if (!frame.IsValid())
            throw new ArgumentException("A geometry snapshot requires an owner frame.", nameof(frame));
        ArgumentNullException.ThrowIfNull(layers);
        ArgumentNullException.ThrowIfNull(boundaries);
        ArgumentNullException.ThrowIfNull(slabs);
        ArgumentNullException.ThrowIfNull(tileChunks);
        OwnerFrame = frame;
        CaptureIdentity = captureIdentity;
        Revision = revision;
        var layerCopy = new Dictionary<LayerId, LayerDefinition>();
        foreach (var (id, definition) in layers)
        {
            definition.Validate();
            layerCopy.Add(id, definition);
        }
        var boundaryCopy = new Dictionary<BoundaryKey, BoundarySegment>();
        var slabCopy = new Dictionary<SurfaceId, SlabSurface>();
        var solids = new List<SpatialSolidPart>();
        foreach (var source in boundaries)
        {
            ArgumentNullException.ThrowIfNull(source);
            if (source.Key.OwnerFrame != frame)
                throw new ArgumentException("A boundary belongs to a different frame.");
            var copy = new BoundarySegment(source.Key, source.Surface, source.Thickness, source.BaseElevation,
                source.Height, source.Openings, source.MaterialProfile, source.Entity, source.StateRevision);
            boundaryCopy.Add(copy.Key, copy);
            AddSurface(copy.Surface, copy.StateRevision, copy.BuildSolidParts(layerCopy[copy.Key.LayerId]));
        }
        foreach (var source in slabs)
        {
            ArgumentNullException.ThrowIfNull(source);
            if (source.Surface.OwnerFrame != frame)
                throw new ArgumentException("A slab belongs to a different frame.");
            var copy = new SlabSurface(source.Surface, source.Layer, source.Footprint, source.TopElevation,
                source.Thickness, source.Openings, source.MaterialProfile, source.Entity, source.StateRevision);
            slabCopy.Add(copy.Surface.Surface, copy);
            AddSurface(copy.Surface.Surface, copy.StateRevision, copy.BuildSolidParts(layerCopy[copy.Layer]));
        }
        var chunkCopy = new Dictionary<SpatialChunkKey, SpatialTileChunk>();
        foreach (var source in tileChunks)
        {
            ArgumentNullException.ThrowIfNull(source);
            if (!layerCopy.ContainsKey(source.Key.Layer))
                throw new ArgumentException("A tile chunk requires an existing authoring layer.");
            if (source.StateRevision > revision)
                throw new ArgumentException("A chunk revision exceeds its geometry snapshot revision.");
            var copy = new SpatialTileChunk(source.Key, source.Tiles, source.StateRevision);
            foreach (var tile in copy.Tiles.Values)
            {
                if (!slabCopy.TryGetValue(tile.StructuralSurface, out var slab) || slab.Layer != copy.Key.Layer)
                    throw new ArgumentException("A floor covering must reference a slab in the same frame and layer.");
            }
            chunkCopy.Add(copy.Key, copy);
        }
        TileChunks = new ReadOnlyDictionary<SpatialChunkKey, SpatialTileChunk>(chunkCopy);
        Layers = new ReadOnlyDictionary<LayerId, LayerDefinition>(layerCopy);
        Boundaries = new ReadOnlyDictionary<BoundaryKey, BoundarySegment>(boundaryCopy);
        Slabs = new ReadOnlyDictionary<SurfaceId, SlabSurface>(slabCopy);
        SolidParts = solids.AsReadOnly();

        void AddSurface(SurfaceId id, ulong stateRevision, IReadOnlyList<SpatialSolidPart> parts)
        {
            if (stateRevision > revision)
                throw new ArgumentException("A surface revision exceeds its geometry snapshot revision.");
            // The same source ID must not identify two unrelated constructions,
            // including a boundary and slab in the same frame.
            _surfaces.Add(id, parts);
            solids.AddRange(parts);
        }
    }

    public bool ContainsSurface(SurfaceId id) => _surfaces.ContainsKey(id);
    public IReadOnlyList<SpatialSolidPart> GetSurfaceParts(SurfaceId id) => _surfaces[id];

    /// <summary>Lookup in known canonical data; network chunk availability is a separate concern.</summary>
    public bool TryGetFloorTile(LayerId layer, Vector2i cell, out SpatialFloorTile tile)
    {
        tile = default;
        return TileChunks.TryGetValue(SpatialChunkKey.FromCell(layer, cell), out var chunk)
            && chunk.TryGetTile(SpatialChunkKey.LocalCell(cell), out tile);
    }
}
