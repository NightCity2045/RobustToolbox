using System;
using Robust.Shared.Serialization;
using Robust.Shared.Serialization.Manager.Attributes;

namespace Robust.Shared.Spatial;

/// <summary>
/// Decorative covering, referencing a structural slab in the same frame and layer.
/// Removing a tile does not remove the slab or change its collision geometry.
/// Material identifiers are opaque to the engine; content supplies their meaning.
/// </summary>
[DataRecord]
public readonly partial record struct SpatialFloorTile : ISerializationHooks
{
    public readonly string MaterialProfile;
    public readonly SurfaceId StructuralSurface;

    public SpatialFloorTile(string materialProfile, SurfaceId structuralSurface)
    {
        MaterialProfile = materialProfile;
        StructuralSurface = structuralSurface;
        Validate();
    }

    public void Validate()
    {
        if (string.IsNullOrWhiteSpace(MaterialProfile))
            throw new ArgumentException("A floor covering requires a material identifier.");
    }

    void ISerializationHooks.AfterDeserialization() => Validate();
}
