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
/// Immutable semantic wall segment. Openings are geometric voids, not material
/// transparency. BaseElevation is relative to the selected authoring layer.
/// </summary>
[DataDefinition]
public sealed partial class BoundarySegment : ISerializationHooks
{
    [DataField("key", required: true)] public BoundaryKey Key { get; private set; }
    [DataField("surface", required: true)] public SurfaceId Surface { get; private set; }
    [DataField("entity")] public EntityUid? Entity { get; private set; }
    [DataField("thickness", required: true)] public float Thickness { get; private set; }
    [DataField("baseElevation")] public float BaseElevation { get; private set; }
    [DataField("height", required: true)] public float Height { get; private set; }
    [DataField("materialProfile")] public string MaterialProfile { get; private set; } = "";
    [DataField("revision")] public ulong StateRevision { get; private set; }
    [DataField("openings")] private BoundaryOpening[] _openings = Array.Empty<BoundaryOpening>();

    public IReadOnlyList<BoundaryOpening> Openings => Array.AsReadOnly(_openings);

    private BoundarySegment() { }

    public BoundarySegment(BoundaryKey key, SurfaceId surface, float thickness, float baseElevation,
        float height, IEnumerable<BoundaryOpening>? openings = null, string materialProfile = "",
        EntityUid? entity = null, ulong stateRevision = 0)
    {
        Key = key;
        Surface = surface;
        Thickness = thickness;
        BaseElevation = baseElevation;
        Height = height;
        MaterialProfile = materialProfile;
        Entity = entity;
        StateRevision = stateRevision;
        _openings = openings?.ToArray() ?? Array.Empty<BoundaryOpening>();
        Validate();
    }

    public void Validate()
    {
        Key.Validate();
        if (!float.IsFinite(Thickness) || Thickness <= 0 || !float.IsFinite(Height) || Height <= 0
            || !float.IsFinite(BaseElevation) || !float.IsFinite(BaseElevation + Height))
            throw new ArgumentException("A boundary requires finite, positive thickness and height.");
        if (MaterialProfile == null || _openings == null)
            throw new ArgumentException("Boundary material and openings cannot be null.");
        if (Entity is { } entity && !entity.IsValid())
            throw new ArgumentException("A source entity reference must be valid or absent.");
        foreach (var opening in _openings)
        {
            opening.Validate();
            if (opening.Top > Height)
                throw new ArgumentException("An opening extends beyond the segment height.");
        }
    }

    void ISerializationHooks.AfterDeserialization() => Validate();

    /// <summary>
    /// Subtract the union of openings from the panel, keeping jambs, sill and lintel.
    /// The caller supplies the definition of Key.LayerId in Key.OwnerFrame.
    /// No GPU, content classes or backend handles are required.
    /// </summary>
    public IReadOnlyList<SpatialSolidPart> BuildSolidParts(LayerDefinition layer)
    {
        Validate();
        layer.Validate();
        var length = layer.AuthoringGrid;
        var rectangles = PlanarRegionPartition.Subtract(new Box2(0, 0, length, Height),
            _openings.Select(o => new Box2(o.Start, o.Bottom, o.End, o.Top)));

        var parts = new List<SpatialSolidPart>(rectangles.Count);
        var originX = (double) Key.X * length;
        var originY = (double) Key.Y * length;
        var originZ = (double) layer.LocalElevation + BaseElevation;
        var half = Thickness / 2d;
        foreach (var rectangle in rectangles)
        {
            var min = Key.Axis == BoundaryAxis.Horizontal
                ? new Vector3((float) (originX + rectangle.Left), (float) (originY - half), (float) (originZ + rectangle.Bottom))
                : new Vector3((float) (originX - half), (float) (originY + rectangle.Left), (float) (originZ + rectangle.Bottom));
            var max = Key.Axis == BoundaryAxis.Horizontal
                ? new Vector3((float) (originX + rectangle.Right), (float) (originY + half), (float) (originZ + rectangle.Top))
                : new Vector3((float) (originX + half), (float) (originY + rectangle.Right), (float) (originZ + rectangle.Top));
            parts.Add(new SpatialSolidPart(new SpatialSurfaceReference(Key.OwnerFrame, Surface), Entity,
                new Box3(min, max), MaterialProfile, StateRevision, Key));
        }
        return parts.AsReadOnly();
    }
}
