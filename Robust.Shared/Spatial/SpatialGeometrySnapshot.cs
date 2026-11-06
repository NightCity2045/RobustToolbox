using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using Robust.Shared.GameObjects;

namespace Robust.Shared.Spatial;

/// <summary>
/// Validated detached snapshot for geometry consumers and worker jobs. Capturing
/// copies semantic definitions; later component deserialization cannot mutate a
/// worker's snapshot. Revision acceptance and installation belong to transactions.
/// </summary>
public sealed class SpatialGeometrySnapshot
{
    public EntityUid OwnerFrame { get; }
    public ulong Revision { get; }
    public IReadOnlyDictionary<LayerId, LayerDefinition> Layers { get; }
    public IReadOnlyDictionary<BoundaryKey, BoundarySegment> Boundaries { get; }
    public IReadOnlyDictionary<SurfaceId, SlabSurface> Slabs { get; }
    public IReadOnlyList<SpatialSolidPart> SolidParts { get; }
    private readonly Dictionary<SurfaceId, IReadOnlyList<SpatialSolidPart>> _surfaces = new();

    internal SpatialGeometrySnapshot(EntityUid frame, ulong revision,
        IReadOnlyDictionary<LayerId, LayerDefinition> layers,
        IEnumerable<BoundarySegment> boundaries, IEnumerable<SlabSurface> slabs)
    {
        if (!frame.IsValid())
            throw new ArgumentException("A geometry snapshot requires an owner frame.", nameof(frame));
        ArgumentNullException.ThrowIfNull(layers);
        ArgumentNullException.ThrowIfNull(boundaries);
        ArgumentNullException.ThrowIfNull(slabs);
        OwnerFrame = frame;
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
}
