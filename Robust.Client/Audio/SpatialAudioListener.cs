using System;
using System.Numerics;
using Robust.Shared.Maths;

namespace Robust.Client.Audio;

/// <summary>
/// Derived listener input in world metres: local +Y forward, +Z up. No entity,
/// camera, portal graph, recipient policy or additional authoritative pose.
/// </summary>
public readonly record struct SpatialAudioListener(RigidTransform3 Pose, Vector3 Velocity)
{
    public Vector3 Forward => Pose.TransformVector(Vector3.UnitY);
    public Vector3 Up => Pose.TransformVector(Vector3.UnitZ);
    internal void Validate()
    {
        if (!Pose.IsValid || !float.IsFinite(Velocity.X) || !float.IsFinite(Velocity.Y) || !float.IsFinite(Velocity.Z))
            throw new ArgumentException("Invalid spatial audio listener pose or velocity.");
    }
}
