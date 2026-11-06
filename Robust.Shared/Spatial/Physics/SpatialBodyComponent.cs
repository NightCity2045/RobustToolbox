using System;
using System.Numerics;
using Robust.Shared.GameObjects;
using Robust.Shared.GameStates;
using Robust.Shared.Serialization;
using Robust.Shared.Serialization.Manager.Attributes;

namespace Robust.Shared.Spatial.Physics;

/// <summary>Canonical spatial velocities and collider definition; pose remains in Transform.</summary>
[RegisterComponent, NetworkedComponent]
public sealed partial class SpatialBodyComponent : Component
{
    [DataField("definition", required: true)] internal SpatialBodyDefinition _definition = default!;
    [DataField("linearVelocity")] internal Vector3 _linearVelocity;
    [DataField("angularVelocity")] internal Vector3 _angularVelocity;
    [DataField("definitionRevision")] internal ulong _definitionRevision;
    public bool HasDefinition => _definition != null;
    public SpatialBodyDefinition Definition => _definition ?? throw new InvalidOperationException("Initial body definition is not available yet.");
    public Vector3 LinearVelocity => _linearVelocity;
    public Vector3 AngularVelocity => _angularVelocity;
    public ulong DefinitionRevision => _definitionRevision;
}

[Serializable, NetSerializable]
public sealed class SpatialBodyComponentState(SpatialBodyDefinition definition, Vector3 linearVelocity, Vector3 angularVelocity,
    ulong definitionRevision = 0) : ComponentState
{
    public readonly SpatialBodyDefinition Definition = definition;
    public readonly Vector3 LinearVelocity = linearVelocity;
    public readonly Vector3 AngularVelocity = angularVelocity;
    public readonly ulong DefinitionRevision = definitionRevision;
}

/// <summary>Explicit physical profile on an existing SpatialWorld map.</summary>
[RegisterComponent, NetworkedComponent]
public sealed partial class SpatialPhysicsWorldComponent : Component
{
    [DataField("settings")] internal SpatialPhysicsSettings _settings = SpatialPhysicsSettings.Default;
    public SpatialPhysicsSettings Settings => _settings;
}

[Serializable, NetSerializable]
public sealed class SpatialPhysicsWorldComponentState(SpatialPhysicsSettings settings) : ComponentState
{
    public readonly SpatialPhysicsSettings Settings = settings;
}
