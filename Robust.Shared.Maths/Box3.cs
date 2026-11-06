using System;
using System.Numerics;

namespace Robust.Shared.Maths;

/// <summary>A finite, positive-volume axis-aligned box in metres.</summary>
[Serializable]
public readonly record struct Box3
{
    public readonly Vector3 Min;
    public readonly Vector3 Max;

    public Box3(Vector3 min, Vector3 max)
    {
        Min = min;
        Max = max;
        if (!IsValid)
            throw new ArgumentException("Box endpoints and dimensions must be finite and strictly ordered.");
    }

    public bool IsValid => Finite(Min) && Finite(Max)
        && Max.X > Min.X && Max.Y > Min.Y && Max.Z > Min.Z && Finite(Max - Min);
    public Vector3 Size => Max - Min;
    public Vector3 Center => Min * 0.5f + Max * 0.5f;
    public double Volume => (double) Size.X * Size.Y * Size.Z;

    public bool Contains(Vector3 point)
        => IsValid && point.X >= Min.X && point.X <= Max.X
            && point.Y >= Min.Y && point.Y <= Max.Y && point.Z >= Min.Z && point.Z <= Max.Z;

    /// <summary>Conservative overlap, including touching faces.</summary>
    public bool Intersects(Box3 other)
        => IsValid && other.IsValid && Min.X <= other.Max.X && Max.X >= other.Min.X
            && Min.Y <= other.Max.Y && Max.Y >= other.Min.Y && Min.Z <= other.Max.Z && Max.Z >= other.Min.Z;

    private static bool Finite(Vector3 value)
        => float.IsFinite(value.X) && float.IsFinite(value.Y) && float.IsFinite(value.Z);
}
