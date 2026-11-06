using System.Numerics;
using Robust.Client.Player;
using Robust.Shared.GameObjects;
using Robust.Shared.IoC;
using Robust.Shared.Maths;

namespace Robust.Client.GameObjects;

public sealed partial class TransformSystem
{
    public override void SetLocalPosition(EntityUid uid, Vector2 value, TransformComponent? xform = null)
    {
        ValidatePosition(value);

        if (!XformQuery.Resolve(uid, ref xform))
            return;

        xform.NextPosition = new Vector3(value, xform._localPosition3.Z);
        ActivateLerp(uid, xform);
        base.SetLocalPosition(uid, value, xform);
    }

    public override void SetLocalRotation(EntityUid uid, Angle value, TransformComponent? xform = null)
    {
        value = NormalizeRotation(value);

        if (!XformQuery.Resolve(uid, ref xform))
            return;

        xform.NextRotation = TransformComponent.WithYaw(xform._localRotation3, value);
        ActivateLerp(uid, xform);
        base.SetLocalRotation(uid, value, xform);
    }

    public override void SetLocalPositionRotation(EntityUid uid, Vector2 pos, Angle rot, TransformComponent? xform = null)
    {
        ValidatePosition(pos);
        rot = NormalizeRotation(rot);

        if (!XformQuery.Resolve(uid, ref xform))
            return;

        xform.NextPosition = new Vector3(pos, xform._localPosition3.Z);
        xform.NextRotation = TransformComponent.WithYaw(xform._localRotation3, rot);
        ActivateLerp(uid, xform);
        base.SetLocalPositionRotation(uid, pos, rot, xform);
    }

    public override void SetLocalPose(EntityUid uid, RigidTransform3 pose, TransformComponent? transform = null)
    {
        ValidatePose(pose);
        if (!XformQuery.Resolve(uid, ref transform))
            return;
        transform.NextPosition = pose.Position;
        transform.NextRotation = pose.Rotation;
        ActivateLerp(uid, transform);
        base.SetLocalPose(uid, pose, transform);
    }

}
