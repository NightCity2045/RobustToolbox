using System;
using System.Globalization;
using Robust.Shared.Serialization;

namespace Robust.Shared.Spatial;

/// <summary>
/// A stable local layer identifier. It is scoped by its owner frame, not a height
/// or a runtime map ID. Copying a prefab remaps the owner frame through the normal
/// entity serializer; the local identifier remains meaningful within that copy.
/// </summary>
[Serializable, NetSerializable]
public record struct LayerId : ISelfSerialize
{
    public readonly uint Value;

    public LayerId(uint value) => Value = value;

    public override string ToString() => Value.ToString(CultureInfo.InvariantCulture);

    string ISelfSerialize.Serialize() => ToString();

    void ISelfSerialize.Deserialize(string value)
        => this = new LayerId(uint.Parse(value, NumberStyles.None, CultureInfo.InvariantCulture));
}
