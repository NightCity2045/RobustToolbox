using System;
using System.Numerics;

namespace Robust.Shared.Maths;

/// <summary>
/// A rigid local-to-parent transform in metres, without scale. The spatial world
/// uses XY as its horizontal plane and +Z as up. This value owns no entity state.
/// </summary>
[Serializable]
public readonly struct RigidTransform3 : IEquatable<RigidTransform3>
{
    public static readonly RigidTransform3 Identity = new(Vector3.Zero, Quaternion.Identity);

    public readonly Vector3 Position;
    public readonly Quaternion Rotation;

    /// <summary>Default-initialized values have an invalid zero quaternion.</summary>
    public bool IsValid => Finite(Position) && Finite(Rotation)
        && MathF.Abs(Rotation.LengthSquared() - 1) <= 1e-5f;

    public RigidTransform3(Vector3 position, Quaternion rotation)
    {
        RequireFinite(position, nameof(position));
        if (!Finite(rotation))
            throw new ArgumentOutOfRangeException(nameof(rotation), "Rotation must be finite.");

        // Double intermediates avoid overflow/underflow for finite scaled inputs.
        var length = Math.Sqrt((double) rotation.X * rotation.X + (double) rotation.Y * rotation.Y
            + (double) rotation.Z * rotation.Z + (double) rotation.W * rotation.W);
        if (length == 0)
            throw new ArgumentOutOfRangeException(nameof(rotation), "Rotation must be nonzero.");

        rotation = new Quaternion((float) (rotation.X / length), (float) (rotation.Y / length),
            (float) (rotation.Z / length), (float) (rotation.W / length));
        // q and -q describe the same orientation; keep one representation.
        if (rotation.W < 0 || rotation.W == 0 &&
            (rotation.X < 0 || rotation.X == 0 &&
                (rotation.Y < 0 || rotation.Y == 0 && rotation.Z < 0)))
            rotation = -rotation;

        // Negative zero must not give equivalent orientations different serialized bytes.
        rotation = new Quaternion(rotation.X == 0 ? 0 : rotation.X, rotation.Y == 0 ? 0 : rotation.Y,
            rotation.Z == 0 ? 0 : rotation.Z, rotation.W == 0 ? 0 : rotation.W);
        Position = position;
        Rotation = rotation;
    }

    public Vector3 TransformPoint(Vector3 localPoint)
    {
        RequireValid();
        RequireFinite(localPoint, nameof(localPoint));
        return Checked(Vector3.Transform(localPoint, Rotation) + Position);
    }

    public Vector3 InverseTransformPoint(Vector3 parentPoint)
    {
        RequireValid();
        RequireFinite(parentPoint, nameof(parentPoint));
        return Checked(Vector3.Transform(parentPoint - Position, Quaternion.Conjugate(Rotation)));
    }

    public Vector3 TransformVector(Vector3 localVector)
    {
        RequireValid();
        RequireFinite(localVector, nameof(localVector));
        return Checked(Vector3.Transform(localVector, Rotation));
    }

    public Vector3 InverseTransformVector(Vector3 parentVector)
    {
        RequireValid();
        RequireFinite(parentVector, nameof(parentVector));
        return Checked(Vector3.Transform(parentVector, Quaternion.Conjugate(Rotation)));
    }

    /// <summary>Apply a child's pose, then this parent's pose.</summary>
    public RigidTransform3 Compose(RigidTransform3 child)
    {
        child.RequireValid();
        return new RigidTransform3(TransformPoint(child.Position),
            Quaternion.Concatenate(child.Rotation, Rotation));
    }

    /// <summary>Express a parent-space pose in this frame, preserving its entire pose.</summary>
    public RigidTransform3 ToLocal(RigidTransform3 parentPose)
    {
        parentPose.RequireValid();
        return new RigidTransform3(InverseTransformPoint(parentPose.Position),
            Quaternion.Concatenate(parentPose.Rotation, Quaternion.Conjugate(Rotation)));
    }

    public RigidTransform3 Inverse()
    {
        RequireValid();
        var inverseRotation = Quaternion.Conjugate(Rotation);
        return new RigidTransform3(Checked(Vector3.Transform(-Position, inverseRotation)), inverseRotation);
    }

    /// <summary>Derived row-vector matrix, matching System.Numerics.Vector3.Transform.</summary>
    public Matrix4x4 ToMatrix()
    {
        RequireValid();
        var result = Matrix4x4.CreateFromQuaternion(Rotation);
        result.Translation = Position;
        return result;
    }

    /// <summary>
    /// Convert a point's frame-relative velocity to parent space. Frame linear and
    /// angular velocities are expressed in parent space (m/s and radians/s).
    /// Includes omega cross r exactly once; it does not move a body or store velocity.
    /// </summary>
    public Vector3 TransformPointVelocity(Vector3 localPoint, Vector3 localVelocity,
        Vector3 frameLinearVelocity, Vector3 frameAngularVelocity)
    {
        RequireFinite(frameLinearVelocity, nameof(frameLinearVelocity));
        RequireFinite(frameAngularVelocity, nameof(frameAngularVelocity));
        return Checked(TransformVector(localVelocity) + frameLinearVelocity
            + Vector3.Cross(frameAngularVelocity, TransformVector(localPoint)));
    }

    /// <summary>Inverse of TransformPointVelocity at the same local point.</summary>
    public Vector3 InverseTransformPointVelocity(Vector3 localPoint, Vector3 parentVelocity,
        Vector3 frameLinearVelocity, Vector3 frameAngularVelocity)
    {
        RequireFinite(parentVelocity, nameof(parentVelocity));
        RequireFinite(frameLinearVelocity, nameof(frameLinearVelocity));
        RequireFinite(frameAngularVelocity, nameof(frameAngularVelocity));
        return InverseTransformVector(Checked(parentVelocity - frameLinearVelocity
            - Vector3.Cross(frameAngularVelocity, TransformVector(localPoint))));
    }

    public bool Equals(RigidTransform3 other) => Position.Equals(other.Position) && Rotation.Equals(other.Rotation);
    public override bool Equals(object? obj) => obj is RigidTransform3 other && Equals(other);
    public override int GetHashCode() => HashCode.Combine(Position, Rotation);

    private void RequireValid()
    {
        if (!IsValid)
            throw new InvalidOperationException("Rigid transform must have a finite position and unit rotation.");
    }

    private static void RequireFinite(Vector3 value, string parameter)
    {
        if (!Finite(value))
            throw new ArgumentOutOfRangeException(parameter, "Vector must be finite.");
    }

    private static Vector3 Checked(Vector3 value)
    {
        if (!Finite(value))
            throw new ArithmeticException("Rigid transform result is not finite.");
        return value;
    }

    private static bool Finite(Vector3 value)
        => float.IsFinite(value.X) && float.IsFinite(value.Y) && float.IsFinite(value.Z);

    private static bool Finite(Quaternion value)
        => float.IsFinite(value.X) && float.IsFinite(value.Y) && float.IsFinite(value.Z) && float.IsFinite(value.W);
}
