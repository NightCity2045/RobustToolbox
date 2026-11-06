using System;
using System.Numerics;
using Robust.Shared.GameObjects;
using Robust.Shared.Serialization;
using Robust.Shared.Serialization.Manager.Attributes;

namespace Robust.Shared.Spatial.Physics;

/// <summary>Neutral control modes. Permission, stamina, jumping and damage belong to content.</summary>
public enum SpatialMotorMode : byte { Walking, Free, Disabled }

/// <summary>Desired world velocity, relative to support while walking. This is an intent, not a pose.</summary>
[Serializable, NetSerializable, DataRecord]
public readonly partial record struct SpatialMotorIntent(Vector3 Velocity, SpatialMotorMode Mode) : ISerializationHooks
{
    public void Validate()
    {
        if (!Enum.IsDefined(Mode) || !SpatialBackendBodyDefinition.Finite(Velocity) || !float.IsFinite(Velocity.LengthSquared()))
            throw new ArgumentException("Motor intent requires a finite velocity and valid mode.");
    }
    void ISerializationHooks.AfterDeserialization() => Validate();
}

[Serializable, NetSerializable, DataRecord]
public readonly partial record struct SpatialMotorSettings(float GroundAcceleration, float AirAcceleration,
    float FreeAcceleration, float MaximumSlope, float StepHeight, float SupportDistance,
    float ContactTolerance) : ISerializationHooks
{
    public static SpatialMotorSettings Default => new(40, 4, 20, MathF.PI / 4, 0.3f, 0.08f, 0.001f);
    public void Validate()
    {
        if (!Nonnegative(GroundAcceleration) || !Nonnegative(AirAcceleration) || !Nonnegative(FreeAcceleration)
            || !Nonnegative(MaximumSlope) || MaximumSlope >= MathF.PI / 2 || !Nonnegative(StepHeight)
            || !float.IsFinite(ContactTolerance) || ContactTolerance <= 0
            || !float.IsFinite(SupportDistance) || SupportDistance < ContactTolerance
            || !float.IsFinite(ContactTolerance * 4 + SupportDistance) || !float.IsFinite(StepHeight + SupportDistance))
            throw new ArgumentException("Invalid motor acceleration, slope or clearance settings.");
    }
    private static bool Nonnegative(float value) => float.IsFinite(value) && value >= 0;
    void ISerializationHooks.AfterDeserialization() => Validate();
}

/// <summary>Derived support witness; canonical body pose and velocity remain in Transform/SpatialBody.</summary>
public readonly record struct SpatialMotorSupport(SpatialQueryCandidate Source, Vector3 Point,
    Vector3 Normal, Vector3 Velocity, float Distance);

[ByRefEvent]
public readonly record struct SpatialMotorSupportChanged(SpatialMotorSupport? Previous, SpatialMotorSupport? Current);

/// <summary>Pre-solve velocity relative to the landed-on surface; content decides consequences.</summary>
[ByRefEvent]
public readonly record struct SpatialMotorLanded(SpatialMotorSupport Support, Vector3 RelativeVelocity);

internal readonly record struct SpatialBackendMotorSupport(SpatialBackendBodyId Body, SpatialBackendPart Source,
    Vector3 Point, Vector3 Normal, Vector3 Velocity, float Distance);
