using System;
using Robust.Shared.GameObjects;
using Robust.Shared.Serialization;
using Robust.Shared.Serialization.Manager.Attributes;

namespace Robust.Shared.Spatial;

/// <summary>
/// Authoring plane relative to its owning frame. Elevation is continuous metres;
/// AuthoringGrid is the horizontal cell size in metres, not a fixed floor height.
/// The owner and local ID are supplied by the containing SpatialFrameComponent.
/// </summary>
[Serializable, NetSerializable, DataRecord]
public readonly partial record struct LayerDefinition : ISerializationHooks
{
    public readonly float LocalElevation;
    public readonly string DisplayName;
    public readonly float AuthoringGrid;

    public LayerDefinition(float localElevation, string displayName, float authoringGrid = 1)
    {
        LocalElevation = localElevation;
        DisplayName = displayName;
        AuthoringGrid = authoringGrid;
        Validate();
    }

    public void Validate()
    {
        if (!float.IsFinite(LocalElevation))
            throw new ArgumentOutOfRangeException(nameof(LocalElevation));
        if (!float.IsFinite(AuthoringGrid) || AuthoringGrid <= 0)
            throw new ArgumentOutOfRangeException(nameof(AuthoringGrid));
        if (DisplayName == null)
            throw new ArgumentNullException(nameof(DisplayName));
    }

    void ISerializationHooks.AfterDeserialization() => Validate();
}

/// <summary>Runtime frame-qualified reference; frame IDs are remapped by the map serializer.</summary>
[DataRecord]
public readonly partial record struct SpatialLayerReference(EntityUid OwnerFrame, LayerId Layer);

/// <summary>Wire equivalent of SpatialLayerReference, with a network entity reference.</summary>
[Serializable, NetSerializable]
public readonly record struct NetSpatialLayerReference(NetEntity OwnerFrame, LayerId Layer);
