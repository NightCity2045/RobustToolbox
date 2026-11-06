using System;
using System.Numerics;
using Robust.Shared.Maths;

namespace Robust.Client.Graphics.Spatial;

/// <summary>Six perspective faces shared by bounded point and spot shadow atlases.</summary>
public static class SpatialShadowProjection
{
    public static int Face(Vector3 fromLight)
    {
        if (!float.IsFinite(fromLight.X) || !float.IsFinite(fromLight.Y) || !float.IsFinite(fromLight.Z))
            throw new ArgumentException("Shadow direction must be finite.");
        var axis = Vector3.Abs(fromLight);
        if (axis.X >= axis.Y && axis.X >= axis.Z) return fromLight.X >= 0 ? 0 : 1;
        if (axis.Y >= axis.Z) return fromLight.Y >= 0 ? 2 : 3;
        return fromLight.Z >= 0 ? 4 : 5;
    }

    public static PerspectiveCamera Camera(Vector3 position, int face, float near, float far)
    {
        var forward = face switch
        {
            0 => Vector3.UnitX, 1 => -Vector3.UnitX, 2 => Vector3.UnitY,
            3 => -Vector3.UnitY, 4 => Vector3.UnitZ, 5 => -Vector3.UnitZ,
            _ => throw new ArgumentOutOfRangeException(nameof(face))
        };
        var nominalUp = face >= 4 ? Vector3.UnitY : Vector3.UnitZ;
        var right = Vector3.Normalize(Vector3.Cross(forward, nominalUp));
        var up = Vector3.Cross(right, forward);
        var basis = new Matrix4x4(right.X, right.Y, right.Z, 0, forward.X, forward.Y, forward.Z, 0,
            up.X, up.Y, up.Z, 0, 0, 0, 0, 1);
        return new PerspectiveCamera(new RigidTransform3(position, Quaternion.CreateFromRotationMatrix(basis)), MathF.PI / 2, 1, near, far);
    }
}
