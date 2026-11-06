using System;
using Robust.Shared.GameObjects;
using Robust.Shared.GameStates;
using Robust.Shared.Serialization;
using Robust.Shared.Serialization.Manager.Attributes;

namespace Robust.Shared.Spatial;

/// <summary>
/// Spatial metadata on the existing map entity. Owns no entity poses or physics
/// handles. Presence of this metadata is not a camera mode or a second map.
/// </summary>
[RegisterComponent, NetworkedComponent]
public sealed partial class SpatialWorldComponent : Component
{
    public const uint CurrentDefinitionSchema = 1;

    [DataField("persistentId")] internal PersistentWorldId _persistentId;
    [DataField("definitionSchema")] internal uint _definitionSchema = CurrentDefinitionSchema;

    public PersistentWorldId PersistentId => _persistentId;
    public uint DefinitionSchema => _definitionSchema;
}

[Serializable, NetSerializable]
public sealed class SpatialWorldComponentState(PersistentWorldId persistentId, uint definitionSchema) : ComponentState
{
    public readonly PersistentWorldId PersistentId = persistentId;
    public readonly uint DefinitionSchema = definitionSchema;
}
