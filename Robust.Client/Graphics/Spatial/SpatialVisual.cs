using System;
using System.Numerics;
using Robust.Client.Utility;
using Robust.Shared.GameObjects;
using Robust.Shared.Graphics.RSI;
using Robust.Shared.Maths;
using Robust.Shared.Spatial;

namespace Robust.Client.Graphics.Spatial;

public enum SpatialVisualShape : byte { Quad, Box }
public enum SpatialSpriteKind : byte { DirectionalBillboard, GroundSprite, SurfaceQuad }

/// <summary>
/// A derived draw record, never a collision body or an additional entity. Quad
/// lies in local XY, facing +Z; Box has unit extents. Scale is physical size.
/// Parts of a primitive assembly share their canonical owner's pose, but may
/// have different local poses and explicit materials. UVs are normalized.
/// </summary>
public sealed record SpatialVisualInstance
{
    public SpatialVisualShape Shape { get; }
    public RigidTransform3 Pose { get; }
    public Vector3 Size { get; }
    public string MaterialProfile { get; }
    public UIBox2 Uv { get; }
    public EntityUid? Entity { get; }
    public SpatialSurfaceReference? Surface { get; }
    public Box3 Bounds { get; }
    public Matrix4x4 Model => Matrix4x4.CreateScale(Size) * Pose.ToMatrix();

    public SpatialVisualInstance(SpatialVisualShape shape, RigidTransform3 pose, Vector3 size, string materialProfile,
        UIBox2? uv = null, EntityUid? entity = null, SpatialSurfaceReference? surface = null)
    {
        var region = uv ?? new UIBox2(0, 0, 1, 1);
        if (!Enum.IsDefined(shape) || !pose.IsValid || !Finite(size) || size.X <= 0 || size.Y <= 0 || size.Z <= 0
            || string.IsNullOrWhiteSpace(materialProfile) || !float.IsFinite(region.Left) || !float.IsFinite(region.Top)
            || !float.IsFinite(region.Right) || !float.IsFinite(region.Bottom) || region.Left < 0 || region.Top < 0
            || region.Right > 1 || region.Bottom > 1 || region.Width <= 0 || region.Height <= 0
            || entity is { } uid && !uid.IsValid())
            throw new ArgumentException("Invalid spatial visual geometry, UV region or identity.");
        Shape = shape; Pose = pose; Size = size; MaterialProfile = materialProfile; Uv = region; Entity = entity; Surface = surface;
        var half = size * 0.5f;
        // Conservative culling bounds only; the rendered quad remains exactly
        // planar and this padding never becomes an authoritative collision shape.
        if (shape == SpatialVisualShape.Quad) half.Z = 0.0001f;
        Bounds = new Box3(-half, half);
    }

    internal static bool Finite(Vector3 v) => float.IsFinite(v.X) && float.IsFinite(v.Y) && float.IsFinite(v.Z);
}

/// <summary>
/// Content-supplied dimensions and pivot measured from the lower-left corner in
/// normalized image coordinates. MaxTilt controls steep-view readability without
/// changing the canonical pose or physical dimensions. Ground/Surface use the
/// supplied attachment's XY plane; billboard up follows canonical local +Z.
/// </summary>
public readonly record struct SpatialSpriteProfile(SpatialSpriteKind Kind, Vector2 Size, Vector2 Pivot, float MaximumTilt)
{
    public SpatialVisualInstance Resolve(RigidTransform3 canonicalPose, PerspectiveCamera camera, string material,
        EntityUid? entity = null, UIBox2? uv = null)
    {
        if (!Enum.IsDefined(Kind) || !canonicalPose.IsValid || !camera.Pose.IsValid
            || !float.IsFinite(Size.X) || !float.IsFinite(Size.Y) || Size.X <= 0 || Size.Y <= 0
            || !float.IsFinite(Pivot.X) || !float.IsFinite(Pivot.Y)
            || !float.IsFinite(MaximumTilt) || MaximumTilt < 0 || MaximumTilt > MathF.PI / 2)
            throw new ArgumentException("Invalid spatial sprite profile or pose.");
        var rotation = canonicalPose.Rotation;
        if (Kind == SpatialSpriteKind.DirectionalBillboard)
        {
            var up = Vector3.Normalize(canonicalPose.TransformVector(Vector3.UnitZ));
            var delta = camera.Pose.Position - canonicalPose.Position;
            var height = Vector3.Dot(delta, up);
            var horizontal = delta - up * height;
            var length = horizontal.Length();
            if (length <= Math.Max(1, delta.Length()) * 1e-5f)
            {
                // At a pole, use the observer's horizontal right vector to avoid
                // an arbitrary changing azimuth or a singular cross product.
                var right = camera.Pose.TransformVector(Vector3.UnitX);
                horizontal = Vector3.Cross(right, up);
                if (horizontal.LengthSquared() < 1e-12f) horizontal = canonicalPose.TransformVector(-Vector3.UnitY);
            }
            var horizontalRight = Vector3.Normalize(Vector3.Cross(up, horizontal));
            horizontal = Vector3.Normalize(Vector3.Cross(horizontalRight, up));
            var tilt = Math.Clamp(MathF.Atan2(height, length), -MaximumTilt, MaximumTilt);
            var normal = horizontal * MathF.Cos(tilt) + up * MathF.Sin(tilt);
            var imageUp = Vector3.Cross(normal, horizontalRight);
            rotation = Basis(horizontalRight, imageUp, normal);
        }
        var offset = new Vector3((0.5f - Pivot.X) * Size.X, (0.5f - Pivot.Y) * Size.Y, 0);
        var pose = new RigidTransform3(canonicalPose.Position + Vector3.Transform(offset, rotation), rotation);
        return new SpatialVisualInstance(SpatialVisualShape.Quad, pose, new Vector3(Size, 1), material, uv, entity);
    }

    internal static Quaternion Basis(Vector3 right, Vector3 up, Vector3 normal)
        => Quaternion.Normalize(Quaternion.CreateFromRotationMatrix(new Matrix4x4(
            right.X, right.Y, right.Z, 0, up.X, up.Y, up.Z, 0,
            normal.X, normal.Y, normal.Z, 0, 0, 0, 0, 1)));
}

/// <summary>Per-observer RSI selection. Entity local -Y is the existing south/front image.</summary>
public static class SpatialSpriteAnimation
{
    public static RsiDirection Direction(RigidTransform3 canonicalPose, PerspectiveCamera observer, RsiDirectionType directions)
    {
        if (!canonicalPose.IsValid || !observer.Pose.IsValid) throw new ArgumentException("Invalid sprite/view pose.");
        var local = canonicalPose.InverseTransformVector(observer.Pose.Position - canonicalPose.Position);
        if (local.X * local.X + local.Y * local.Y <= Math.Max(1, local.LengthSquared()) * 1e-10f)
            local = canonicalPose.InverseTransformVector(observer.Pose.TransformVector(-Vector3.UnitY));
        return new Angle(Math.Atan2(local.X, -local.Y)).ToRsiDirection(directions);
    }

    /// <summary>Sample folded RSI delays without advancing or mutating a shared SpriteComponent.</summary>
    public static int Frame(IRsiStateLike state, double seconds, bool loop = true)
    {
        ArgumentNullException.ThrowIfNull(state);
        if (!double.IsFinite(seconds) || seconds < 0) throw new ArgumentOutOfRangeException(nameof(seconds));
        if (!state.IsAnimated) return 0;
        var count = state.AnimationFrameCount;
        if (count <= 0) throw new ArgumentException("Animated state has no frames.");
        double duration = 0;
        for (var i = 0; i < count; i++)
        {
            var delay = state.GetDelay(i);
            if (!float.IsFinite(delay) || delay <= 0) throw new ArgumentException("Animation delays must be finite and positive.");
            duration += delay;
        }
        if (!loop && seconds >= duration) return count - 1;
        var time = loop ? seconds % duration : seconds;
        for (var i = 0; i < count - 1; i++)
        {
            var delay = state.GetDelay(i);
            if (time < delay) return i;
            time -= delay;
        }
        return count - 1;
    }
}

/// <summary>
/// Decal coordinates are on a particular exterior patch: (Y,Z), (X,Z), or
/// (X,Y) for X/Y/Z normals. Resolve against the current snapshot and frame pose.
/// A removed/revised surface or an opening invalidates the attachment explicitly.
/// </summary>
public readonly record struct SpatialDecalAttachment(SpatialSurfaceReference Surface, ulong Revision,
    SpatialFaceDirection Face, float Plane, Box2 Region, float NormalOffset)
{
    public bool TryResolve(SpatialGeometrySnapshot snapshot, RigidTransform3 framePose, string material, out SpatialVisualInstance? visual)
    {
        visual = null;
        if (!framePose.IsValid || !Enum.IsDefined(Face) || !float.IsFinite(Plane) || !float.IsFinite(NormalOffset)
            || NormalOffset < 0 || !float.IsFinite(Region.Left) || !float.IsFinite(Region.Right)
            || !float.IsFinite(Region.Top) || !float.IsFinite(Region.Bottom) || Region.Width <= 0 || Region.Height <= 0)
            throw new ArgumentException("Invalid decal attachment.");
        if (snapshot.OwnerFrame != Surface.OwnerFrame) return false;
        var reference = Surface;
        foreach (var patch in SpatialSurfaceBuilder.Enumerate(snapshot, p => p.Surface == reference, default))
        {
            if (patch.Source.Surface != Surface || patch.Source.Revision != Revision || patch.Direction != Face || patch.Plane != Plane
                || Region.Left < patch.Region.Left || Region.Right > patch.Region.Right
                || Region.Bottom < patch.Region.Bottom || Region.Top > patch.Region.Top) continue;
            var axis = (int) Face / 2;
            var center = Region.Center;
            var point = axis switch { 0 => new Vector3(Plane, center.X, center.Y), 1 => new Vector3(center.X, Plane, center.Y), _ => new Vector3(center, Plane) };
            var right = axis == 0 ? Vector3.UnitY : Vector3.UnitX;
            var up = Vector3.Cross(patch.Normal, right);
            var rotation = SpatialSpriteProfile.Basis(right, up, patch.Normal);
            var local = new RigidTransform3(point + patch.Normal * NormalOffset, rotation);
            visual = new SpatialVisualInstance(SpatialVisualShape.Quad, framePose.Compose(local),
                new Vector3(Region.Width, Region.Height, 1), material, entity: patch.Source.Entity, surface: Surface);
            return true;
        }
        return false;
    }
}
