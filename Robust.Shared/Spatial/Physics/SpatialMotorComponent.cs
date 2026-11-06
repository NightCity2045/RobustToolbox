using System;
using System.Numerics;
using Robust.Shared.GameObjects;
using Robust.Shared.GameStates;
using Robust.Shared.Serialization;
using Robust.Shared.Serialization.Manager.Attributes;

namespace Robust.Shared.Spatial.Physics;

/// <summary>Control parameters and intent only; Transform and SpatialBody own all motion.</summary>
[RegisterComponent, NetworkedComponent]
public sealed partial class SpatialMotorComponent : Component
{
    [DataField("settings")] internal SpatialMotorSettings _settings = SpatialMotorSettings.Default;
    [DataField("intent")] internal SpatialMotorIntent _intent;
    internal SpatialMotorSupport? _support;
    public SpatialMotorSettings Settings => _settings;
    public SpatialMotorIntent Intent => _intent;
    public SpatialMotorSupport? Support => _support;
}

[Serializable, NetSerializable]
public sealed record SpatialMotorSupportState(NetEntity? Entity, NetEntity? Frame, SurfaceId? Surface,
    string MaterialProfile, ulong Revision, Vector3 Point, Vector3 Normal, Vector3 Velocity, float Distance);

[Serializable, NetSerializable]
public sealed class SpatialMotorComponentState(SpatialMotorSettings settings, SpatialMotorIntent intent,
    SpatialMotorSupportState? support) : ComponentState
{
    public readonly SpatialMotorSettings Settings = settings;
    public readonly SpatialMotorIntent Intent = intent;
    public readonly SpatialMotorSupportState? Support = support;
}
