using System;
using System.Numerics;
using Robust.Shared.Maths;

namespace Robust.Shared.GameObjects;

public abstract partial class SharedTransformSystem
{
    /// <summary>Compose the canonical local poses through the existing parent hierarchy.</summary>
    public RigidTransform3 GetWorldPose(EntityUid uid) => GetWorldPose(XformQuery.GetComponent(uid));

    public RigidTransform3 GetWorldPose(TransformComponent transform)
    {
        var pose = transform.LocalPose;
        var parent = transform.ParentUid;
        while (parent.IsValid())
        {
            var parentTransform = XformQuery.GetComponent(parent);
            pose = parentTransform.LocalPose.Compose(pose);
            parent = parentTransform.ParentUid;
        }
        return pose;
    }

    /// <summary>Express a point in another entity frame without projecting away height.</summary>
    public Vector3 ConvertPoint(EntityUid sourceFrame, EntityUid targetFrame, Vector3 point)
    {
        RequireSameWorld(sourceFrame, targetFrame);
        return GetWorldPose(targetFrame).InverseTransformPoint(GetWorldPose(sourceFrame).TransformPoint(point));
    }

    /// <summary>Express the complete pose in an explicitly selected frame of the same world.</summary>
    public RigidTransform3 GetPoseRelativeTo(EntityUid uid, EntityUid frame)
    {
        RequireSameWorld(uid, frame);
        return GetWorldPose(frame).ToLocal(GetWorldPose(uid));
    }

    public Matrix4x4 GetWorldMatrix3(EntityUid uid) => GetWorldPose(uid).ToMatrix();
    public Matrix4x4 GetInvWorldMatrix3(EntityUid uid) => GetWorldPose(uid).Inverse().ToMatrix();

    /// <summary>Explicit projection onto the selected frame's XY plane.</summary>
    public Vector2 GetPlanarPosition(EntityUid uid, EntityUid planeFrame)
    {
        var position = GetPoseRelativeTo(uid, planeFrame).Position;
        return new Vector2(position.X, position.Y);
    }

    /// <summary>
    /// Change coordinates in a selected frame's XY plane while preserving signed
    /// depth in that frame and the entity's complete world orientation. A tilted
    /// plane's normal need not be the world's +Z axis.
    /// </summary>
    public void SetPlanarPosition(EntityUid uid, EntityUid planeFrame, Vector2 position)
    {
        ValidatePosition(position);
        var relative = GetPoseRelativeTo(uid, planeFrame);
        var updated = new RigidTransform3(new Vector3(position, relative.Position.Z), relative.Rotation);
        SetWorldPose(uid, GetWorldPose(planeFrame).Compose(updated));
    }

    private void RequireSameWorld(EntityUid first, EntityUid second)
    {
        if (Root(first) != Root(second))
            throw new InvalidOperationException("Coordinate conversion requires frames in the same world hierarchy.");
    }

    private EntityUid Root(EntityUid uid)
    {
        var transform = XformQuery.GetComponent(uid);
        while (transform.ParentUid.IsValid())
        {
            uid = transform.ParentUid;
            transform = XformQuery.GetComponent(uid);
        }
        return uid;
    }

    public virtual void SetLocalPose(EntityUid uid, RigidTransform3 pose, TransformComponent? transform = null)
        => SetLocalPoseNoLerp(uid, pose, transform);

    public void SetLocalPoseNoLerp(EntityUid uid, RigidTransform3 pose, TransformComponent? transform = null)
    {
        ValidatePose(pose);
        if (!XformQuery.Resolve(uid, ref transform))
            return;

        var oldPose = transform.LocalPose;
        var position = transform.Anchored ? oldPose.Position : pose.Position;
        var rotation = transform.NoLocalRotation ? oldPose.Rotation : pose.Rotation;
        var positionChanged = position != oldPose.Position;
        var rotationChanged = rotation != oldPose.Rotation;
        if (!positionChanged && !rotationChanged)
            return;

        var meta = MetaData(uid);
        transform._localPosition3 = position;
        transform._localRotation3 = rotation;
        transform.MatricesDirty = true;
        if (positionChanged)
            DirtyField(uid, transform, nameof(TransformComponent.LocalPosition), meta);
        if (rotationChanged)
            DirtyField(uid, transform, nameof(TransformComponent.LocalRotation), meta);

        if (transform.Initialized)
            RaiseMoveEvent((uid, transform, meta), transform.ParentUid, oldPose, transform.MapUid,
                checkTraversal: positionChanged);
    }

    public void SetWorldPose(EntityUid uid, RigidTransform3 pose, TransformComponent? transform = null)
    {
        ValidatePose(pose);
        if (!XformQuery.Resolve(uid, ref transform))
            return;
        var local = transform.ParentUid.IsValid() ? GetWorldPose(transform.ParentUid).ToLocal(pose) : pose;
        SetLocalPose(uid, local, transform);
    }

    /// <summary>Set a complete pose relative to a specified parent, using the same hierarchy mutation path.</summary>
    public void SetParentLocalPose(EntityUid uid, EntityUid parent, RigidTransform3 pose,
        TransformComponent? transform = null)
    {
        ValidatePose(pose);
        if (!XformQuery.Resolve(uid, ref transform))
            return;
        SetCoordinates3((uid, transform, MetaData(uid)), parent, pose.Position, pose.Rotation);
    }

    protected static void ValidatePose(RigidTransform3 pose)
    {
        if (!pose.IsValid)
            throw new ArgumentOutOfRangeException(nameof(pose), "Transform pose must have a finite position and unit orientation.");
    }
}
