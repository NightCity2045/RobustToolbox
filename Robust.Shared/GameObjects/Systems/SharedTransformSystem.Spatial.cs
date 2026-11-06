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
        => GetWorldPose(targetFrame).InverseTransformPoint(GetWorldPose(sourceFrame).TransformPoint(point));

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
