using System;
using Robust.Shared.Serialization;

namespace Robust.Shared.Spatial;

/// <summary>Persistent map identity, separate from MapId, EntityUid and NetEntity.</summary>
[Serializable, NetSerializable]
public record struct PersistentWorldId : ISelfSerialize
{
    public readonly Guid Value;
    public bool IsValid => Value != Guid.Empty;

    public PersistentWorldId(Guid value)
    {
        if (value == Guid.Empty)
            throw new ArgumentException("A persistent world ID cannot be empty.", nameof(value));
        Value = value;
    }

    public static PersistentWorldId New() => new(Guid.NewGuid());
    public override string ToString() => Value.ToString("D");
    string ISelfSerialize.Serialize() => IsValid ? ToString() : throw new InvalidOperationException("Unassigned world ID.");
    void ISelfSerialize.Deserialize(string value) => this = new PersistentWorldId(Guid.ParseExact(value, "D"));
}
