using System;
using System.Numerics;
using Robust.Shared.Maths;

namespace Robust.Shared.Spatial.Physics;

internal readonly record struct SpatialBackendConstraintId(ulong Value);
public enum SpatialConstraintKind : byte { Fixed, Point, DistanceRange }

/// <summary>Attachment offsets and fixed relative pose are relative to canonical entity origins.</summary>
internal sealed record SpatialBackendConstraintDefinition(SpatialConstraintKind Kind,
    SpatialBackendBodyId BodyA, SpatialBackendBodyId BodyB, Vector3 LocalAnchorA, Vector3 LocalAnchorB,
    RigidTransform3 RelativePose, float MinimumDistance = 0, float MaximumDistance = 0,
    float SpringFrequency = 30, float Damping = 1, bool CollideConnected = false)
{
    public void Validate()
    {
        if (BodyA.Value == 0 || BodyB.Value == 0 || BodyA == BodyB)
            throw new ArgumentException("Invalid spatial constraint definition.");
        ValidateSettings(Kind, LocalAnchorA, LocalAnchorB, RelativePose, MinimumDistance, MaximumDistance, SpringFrequency, Damping);
    }

    internal static void ValidateSettings(SpatialConstraintKind kind, Vector3 anchorA, Vector3 anchorB,
        RigidTransform3 relative, float minimum, float maximum, float frequency, float damping)
    {
        var angularFrequency = 2 * MathF.PI * frequency;
        if (!Enum.IsDefined(kind) || !float.IsFinite(anchorA.LengthSquared()) || !float.IsFinite(anchorB.LengthSquared())
            || !relative.IsValid || !float.IsFinite(relative.Position.LengthSquared())
            || !float.IsFinite(minimum) || minimum < 0 || !float.IsFinite(maximum * maximum) || maximum < minimum
            || !float.IsFinite(frequency) || frequency <= 0 || !float.IsFinite(damping) || damping < 0
            || !float.IsFinite(angularFrequency * angularFrequency) || !float.IsFinite(2 * damping * angularFrequency))
            throw new ArgumentException("Constraint settings exceed the finite solver range.");
    }
}
