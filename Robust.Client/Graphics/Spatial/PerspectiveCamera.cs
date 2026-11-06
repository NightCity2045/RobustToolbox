using System;
using System.Numerics;
using Robust.Shared.Maths;

namespace Robust.Client.Graphics.Spatial;

/// <summary>A world-space picking ray with distances to the actual near/far planes.</summary>
public readonly record struct SpatialCameraRay(Vector3 Origin, Vector3 Direction, float MinimumDistance, float MaximumDistance);

/// <summary>
/// Immutable camera value. Identity faces local +Y, with +X right and +Z up.
/// Matrices use System.Numerics row vectors and OpenGL clip depth [-1,+1].
/// This view never owns or changes the observed entity's canonical transform.
/// </summary>
public readonly struct PerspectiveCamera
{
    public RigidTransform3 Pose { get; }
    public float VerticalFieldOfView { get; }
    public float AspectRatio { get; }
    public float Near { get; }
    public float Far { get; }
    public Matrix4x4 View { get; }
    public Matrix4x4 Projection { get; }
    public Matrix4x4 ViewProjection { get; }
    public SpatialViewFrustum Frustum { get; }
    public Vector3 Right => Pose.TransformVector(Vector3.UnitX);
    public Vector3 Forward => Pose.TransformVector(Vector3.UnitY);
    public Vector3 Up => Pose.TransformVector(Vector3.UnitZ);

    public PerspectiveCamera(RigidTransform3 pose, float verticalFieldOfView, float aspectRatio, float near, float far)
    {
        if (!pose.IsValid || !float.IsFinite(verticalFieldOfView) || verticalFieldOfView <= 0 || verticalFieldOfView >= MathF.PI
            || !float.IsFinite(aspectRatio) || aspectRatio <= 0 || !float.IsFinite(near) || near <= 0
            || !float.IsFinite(far) || far <= near)
            throw new ArgumentException("A camera requires a valid pose, FOV, aspect ratio and finite ordered clip distances.");
        Pose = pose;
        VerticalFieldOfView = verticalFieldOfView;
        AspectRatio = aspectRatio;
        Near = near;
        Far = far;
        var right = pose.TransformVector(Vector3.UnitX);
        var up = pose.TransformVector(Vector3.UnitZ);
        var forward = pose.TransformVector(Vector3.UnitY);
        // Build from the orientation basis, not position + forward: that sum
        // loses the direction at large translations even with a valid quaternion.
        View = new Matrix4x4(right.X, up.X, -forward.X, 0,
            right.Y, up.Y, -forward.Y, 0, right.Z, up.Z, -forward.Z, 0,
            -Vector3.Dot(pose.Position, right), -Vector3.Dot(pose.Position, up), Vector3.Dot(pose.Position, forward), 1);
        var cotangent = 1 / Math.Tan(verticalFieldOfView * 0.5);
        var range = (double) far - near;
        Projection = new Matrix4x4((float) (cotangent / aspectRatio), 0, 0, 0,
            0, (float) cotangent, 0, 0, 0, 0, (float) (-((double) far + near) / range), -1,
            0, 0, (float) (-2 * (double) near * far / range), 0);
        ViewProjection = View * Projection;
        Frustum = new SpatialViewFrustum(ViewProjection);
    }

    /// <summary>Project to top-left normalized screen coordinates and OpenGL depth. Outside-screen points are allowed.</summary>
    public bool TryProject(Vector3 world, out Vector3 screen)
    {
        screen = default;
        if (!Pose.IsValid || !float.IsFinite(world.X) || !float.IsFinite(world.Y) || !float.IsFinite(world.Z)) return false;
        var clip = Vector4.Transform(new Vector4(world, 1), ViewProjection);
        if (!float.IsFinite(clip.W) || clip.W <= 0) return false;
        var ndc = new Vector3(clip.X, clip.Y, clip.Z) / clip.W;
        if (!float.IsFinite(ndc.X) || !float.IsFinite(ndc.Y) || !float.IsFinite(ndc.Z)) return false;
        screen = new Vector3((ndc.X + 1) * 0.5f, (1 - ndc.Y) * 0.5f, ndc.Z);
        return true;
    }

    /// <summary>Picking uses the same perspective projection, independent of render resolution and UI scale.</summary>
    public SpatialCameraRay GetRay(Vector2 normalizedScreen)
    {
        if (!Pose.IsValid || AspectRatio <= 0 || !float.IsFinite(normalizedScreen.X) || !float.IsFinite(normalizedScreen.Y)
            || normalizedScreen.X < 0 || normalizedScreen.X > 1 || normalizedScreen.Y < 0 || normalizedScreen.Y > 1)
            throw new ArgumentException("Picking requires a valid camera and normalized viewport position.");
        var tangent = MathF.Tan(VerticalFieldOfView * 0.5f);
        var local = Vector3.Normalize(new Vector3((normalizedScreen.X * 2 - 1) * tangent * AspectRatio,
            1, (1 - normalizedScreen.Y * 2) * tangent));
        var direction = Pose.TransformVector(local);
        var minimum = Near / local.Y;
        var maximum = Far / local.Y;
        if (!float.IsFinite(maximum) || !float.IsFinite(minimum))
            throw new ArithmeticException("Camera ray clip distance exceeds the finite range.");
        return new SpatialCameraRay(Pose.Position, direction, minimum, maximum);
    }
}

/// <summary>Six inward-facing planes. Intersections include geometry touching the clip planes.</summary>
public readonly struct SpatialViewFrustum
{
    private readonly Vector4 _left, _right, _bottom, _top, _near, _far;

    internal SpatialViewFrustum(Matrix4x4 matrix)
    {
        var x = new Vector4(matrix.M11, matrix.M21, matrix.M31, matrix.M41);
        var y = new Vector4(matrix.M12, matrix.M22, matrix.M32, matrix.M42);
        var z = new Vector4(matrix.M13, matrix.M23, matrix.M33, matrix.M43);
        var w = new Vector4(matrix.M14, matrix.M24, matrix.M34, matrix.M44);
        _left = Normalize(w + x); _right = Normalize(w - x);
        _bottom = Normalize(w + y); _top = Normalize(w - y);
        _near = Normalize(w + z); _far = Normalize(w - z);
    }

    public bool Intersects(Box3 worldBounds) => Intersects(worldBounds, RigidTransform3.Identity);

    /// <summary>Test a frame-local chunk without inflating a rotated vehicle into a world-aligned box.</summary>
    public bool Intersects(Box3 localBounds, RigidTransform3 framePose)
    {
        if (!localBounds.IsValid || !framePose.IsValid || _near == Vector4.Zero)
            throw new ArgumentException("Frustum intersection requires valid bounds, pose and camera.");
        var center = localBounds.Min * 0.5f + localBounds.Max * 0.5f;
        var half = localBounds.Max * 0.5f - localBounds.Min * 0.5f;
        center = framePose.TransformPoint(center);
        var x = framePose.TransformVector(new Vector3(half.X, 0, 0));
        var y = framePose.TransformVector(new Vector3(0, half.Y, 0));
        var z = framePose.TransformVector(new Vector3(0, 0, half.Z));
        return Visible(_left) && Visible(_right) && Visible(_bottom) && Visible(_top) && Visible(_near) && Visible(_far);

        bool Visible(Vector4 plane)
        {
            var distance = Dot(plane, center) + plane.W;
            var radius = Math.Abs(Dot(plane, x)) + Math.Abs(Dot(plane, y)) + Math.Abs(Dot(plane, z));
            return distance + radius >= -1e-5;
        }
    }

    private static double Dot(Vector4 plane, Vector3 point)
        => (double) plane.X * point.X + (double) plane.Y * point.Y + (double) plane.Z * point.Z;

    private static Vector4 Normalize(Vector4 plane)
    {
        var length = Math.Sqrt((double) plane.X * plane.X + (double) plane.Y * plane.Y + (double) plane.Z * plane.Z);
        if (!double.IsFinite(length) || length <= 0 || !float.IsFinite(plane.W))
            throw new ArgumentException("Camera projection has an unrepresentable frustum.");
        var result = new Vector4((float) (plane.X / length), (float) (plane.Y / length),
            (float) (plane.Z / length), (float) (plane.W / length));
        if (!float.IsFinite(result.W)) throw new ArgumentException("Camera clip plane exceeds the finite range.");
        return result;
    }
}
