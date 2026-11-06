using System;
using System.Collections.Generic;
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

    /// <summary>
    /// Commit a physics tick before notifying move observers. Parent and child
    /// local poses are derived from the complete set of final world poses, so an
    /// observer never sees a free body carried temporarily by its moving parent.
    /// The trusted caller commits already-validated velocities before notifications.
    /// </summary>
    internal void CommitSpatialPoses(IReadOnlyDictionary<EntityUid, RigidTransform3> poses, Action commitVelocities)
    {
        var resolved = new Dictionary<EntityUid, RigidTransform3>();
        var plans = new List<(Entity<TransformComponent, MetaDataComponent> Entity, RigidTransform3 Old, RigidTransform3 Next,
            bool PositionChanged, bool RotationChanged)>();
        foreach (var (uid, pose) in poses)
        {
            ValidatePose(pose);
            if (TerminatingOrDeleted(uid))
                throw new InvalidOperationException("A terminating entity cannot receive a physics pose.");
            var xform = Transform(uid);
            var local = xform.ParentUid.IsValid() ? ResolveFinalPose(xform.ParentUid).ToLocal(pose) : pose;
            var old = xform.LocalPose;
            var positionChanged = old.Position != local.Position;
            var rotationChanged = old.Rotation != local.Rotation;
            if (positionChanged && xform.Anchored || rotationChanged && xform.NoLocalRotation)
                throw new InvalidOperationException("Physics cannot override another transform movement owner.");
            if (positionChanged || rotationChanged)
                plans.Add(((uid, xform, MetaData(uid)), old, local, positionChanged, rotationChanged));
        }

        // No mutation or external callbacks occurred during preflight.
        foreach (var plan in plans)
        {
            var (uid, xform, meta) = plan.Entity;
            xform._localPosition3 = plan.Next.Position;
            xform._localRotation3 = plan.Next.Rotation;
            xform.MatricesDirty = true;
            if (plan.PositionChanged)
                DirtyField(uid, xform, nameof(TransformComponent.LocalPosition), meta);
            if (plan.RotationChanged)
                DirtyField(uid, xform, nameof(TransformComponent.LocalRotation), meta);
        }
        commitVelocities();
        foreach (var plan in plans)
        {
            if (TerminatingOrDeleted(plan.Entity.Owner) || !plan.Entity.Comp1.Initialized)
                continue;
            // Spatial support/frame selection does not use legacy planar grid traversal.
            RaiseMoveEvent(plan.Entity, plan.Entity.Comp1.ParentUid, plan.Old, plan.Entity.Comp1.MapUid, checkTraversal: false);
        }

        RigidTransform3 ResolveFinalPose(EntityUid uid)
        {
            var pending = new Stack<EntityUid>();
            var pose = RigidTransform3.Identity;
            while (uid.IsValid())
            {
                if (poses.TryGetValue(uid, out pose) || resolved.TryGetValue(uid, out pose))
                    break;
                pending.Push(uid);
                uid = Transform(uid).ParentUid;
            }
            if (!uid.IsValid())
                pose = RigidTransform3.Identity;
            while (pending.TryPop(out var ancestor))
            {
                pose = pose.Compose(Transform(ancestor).LocalPose);
                resolved[ancestor] = pose;
            }
            return pose;
        }
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

    /// <summary>Apply a spawn pose before component initialization, retaining prototype anchoring.</summary>
    internal void SetInitialPose(Entity<TransformComponent, MetaDataComponent> entity, EntityUid parent,
        RigidTransform3 pose)
    {
        ValidatePose(pose);
        if (entity.Comp2.EntityLifeStage >= EntityLifeStage.Initializing)
            throw new InvalidOperationException("An initial pose must be assigned before entity initialization.");
        if (entity.Comp1.NoLocalRotation && pose.Rotation != Quaternion.Identity)
            throw new InvalidOperationException("An entity with NoLocalRotation cannot spawn with a rotated local pose.");
        SetCoordinates3(entity, parent, pose.Position, pose.Rotation, unanchor: false);
    }

    protected static void ValidatePose(RigidTransform3 pose)
    {
        if (!pose.IsValid)
            throw new ArgumentOutOfRangeException(nameof(pose), "Transform pose must have a finite position and unit orientation.");
    }
}
