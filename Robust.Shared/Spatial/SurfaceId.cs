using System;
using System.Globalization;
using Robust.Shared.GameObjects;
using Robust.Shared.Serialization;
using Robust.Shared.Serialization.Manager.Attributes;

namespace Robust.Shared.Spatial;

/// <summary>A stable surface identifier scoped by its owner frame, independent of mesh/physics indices.</summary>
[Serializable, NetSerializable]
public record struct SurfaceId : ISelfSerialize
{
    public readonly uint Value;
    public SurfaceId(uint value) => Value = value;
    public override string ToString() => Value.ToString(CultureInfo.InvariantCulture);
    string ISelfSerialize.Serialize() => ToString();
    void ISelfSerialize.Deserialize(string value)
        => this = new SurfaceId(uint.Parse(value, NumberStyles.None, CultureInfo.InvariantCulture));
}

[DataRecord]
public readonly partial record struct SpatialSurfaceReference(EntityUid OwnerFrame, SurfaceId Surface);
