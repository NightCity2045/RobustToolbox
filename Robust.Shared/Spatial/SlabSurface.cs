using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using Robust.Shared.GameObjects;
using Robust.Shared.Maths;
using Robust.Shared.Serialization;
using Robust.Shared.Serialization.Manager.Attributes;

namespace Robust.Shared.Spatial;

/// <summary>
/// Structural slab with actual thickness and through-openings. Its top is relative
/// to its authoring layer; the solid extends downward. Decorative floor tiles are
/// separate data and do not determine whether this structural surface exists.
/// </summary>
[DataDefinition]
public sealed partial class SlabSurface : ISerializationHooks
{
    [DataField("surface", required: true)] public SpatialSurfaceReference Surface { get; private set; }
    [DataField("layer", required: true)] public LayerId Layer { get; private set; }
    [DataField("footprint", required: true)] public Box2 Footprint { get; private set; }
    [DataField("topElevation")] public float TopElevation { get; private set; }
    [DataField("thickness", required: true)] public float Thickness { get; private set; }
    [DataField("entity")] public EntityUid? Entity { get; private set; }
    [DataField("materialProfile")] public string MaterialProfile { get; private set; } = "";
    [DataField("revision")] public ulong StateRevision { get; private set; }
    [DataField("openings")] private Box2[] _openings = Array.Empty<Box2>();

    public IReadOnlyList<Box2> Openings => Array.AsReadOnly(_openings);
    private SlabSurface() { }

    public SlabSurface(SpatialSurfaceReference surface, LayerId layer, Box2 footprint, float topElevation,
        float thickness, IEnumerable<Box2>? openings = null, string materialProfile = "",
        EntityUid? entity = null, ulong stateRevision = 0)
    {
        Surface = surface;
        Layer = layer;
        Footprint = footprint;
        TopElevation = topElevation;
        Thickness = thickness;
        MaterialProfile = materialProfile;
        Entity = entity;
        StateRevision = stateRevision;
        _openings = openings?.ToArray() ?? Array.Empty<Box2>();
        Validate();
    }

    public void Validate()
    {
        if (!Surface.OwnerFrame.IsValid() || Entity is { } entity && !entity.IsValid())
            throw new ArgumentException("A slab requires a valid owner and an optional valid source entity.");
        PlanarRegionPartition.ValidateRectangle(Footprint);
        if (!float.IsFinite(TopElevation) || !float.IsFinite(Thickness) || Thickness <= 0
            || !float.IsFinite(TopElevation - Thickness))
            throw new ArgumentException("A slab requires finite elevation and positive thickness.");
        if (MaterialProfile == null || _openings == null)
            throw new ArgumentException("Slab material and openings cannot be null.");
        foreach (var opening in _openings)
            PlanarRegionPartition.ValidateOpening(Footprint, opening);
    }

    void ISerializationHooks.AfterDeserialization() => Validate();

    public IReadOnlyList<SpatialSolidPart> BuildSolidParts(LayerDefinition layer)
    {
        Validate();
        layer.Validate();
        var rectangles = PlanarRegionPartition.Subtract(Footprint, _openings);
        var top = (double) layer.LocalElevation + TopElevation;
        var bottom = top - Thickness;
        var result = new List<SpatialSolidPart>(rectangles.Count);
        foreach (var rectangle in rectangles)
        {
            var bounds = new Box3(new Vector3(rectangle.Left, rectangle.Bottom, (float) bottom),
                new Vector3(rectangle.Right, rectangle.Top, (float) top));
            result.Add(new SpatialSolidPart(Surface, Entity, bounds, MaterialProfile, StateRevision));
        }
        return result.AsReadOnly();
    }
}
