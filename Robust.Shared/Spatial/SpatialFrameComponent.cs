using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using Robust.Shared.GameObjects;
using Robust.Shared.GameStates;
using Robust.Shared.Serialization;
using Robust.Shared.Serialization.Manager.Attributes;

namespace Robust.Shared.Spatial;

/// <summary>Layer definitions owned by an existing transform frame, never another pose.</summary>
[RegisterComponent, NetworkedComponent]
public sealed partial class SpatialFrameComponent : Component
{
    [DataField("layers")] internal Dictionary<LayerId, LayerDefinition> _layers = new();

    public IReadOnlyDictionary<LayerId, LayerDefinition> Layers => new ReadOnlyDictionary<LayerId, LayerDefinition>(_layers);
}

[Serializable, NetSerializable]
public sealed class SpatialFrameComponentState(Dictionary<LayerId, LayerDefinition> layers) : ComponentState
{
    public readonly Dictionary<LayerId, LayerDefinition> Layers = new(layers);
}
