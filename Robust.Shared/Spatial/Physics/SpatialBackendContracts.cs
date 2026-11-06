using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using Robust.Shared.GameObjects;
using Robust.Shared.Maths;

namespace Robust.Shared.Spatial.Physics;

// Trusted engine contracts. No BEPU types cross this boundary. The ECS/query
// services expose world/frame-aware operations, not these low-level body tokens.
internal readonly record struct SpatialBackendBodyId(ulong Value);
internal enum SpatialBackendMotion { Static, Dynamic, Kinematic }
internal abstract record SpatialConvexShape;
internal sealed record SpatialBox(Vector3 Size) : SpatialConvexShape;
internal sealed record SpatialSphere(float Radius) : SpatialConvexShape;
/// <summary>Capsule along local +Z; Length excludes the two hemispheres.</summary>
internal sealed record SpatialCapsule(float Radius, float Length) : SpatialConvexShape;

internal readonly record struct SpatialBackendPart(SpatialConvexShape Shape, RigidTransform3 LocalPose,
    EntityUid? Entity = null, SpatialSurfaceReference? Surface = null, ulong Revision = 0, float MassWeight = 1,
    string MaterialProfile = "", uint? CollisionLayer = null, uint? CollisionMask = null);

internal readonly record struct SpatialBackendMaterial(float Friction, float RecoveryVelocity, float SpringFrequency, float Damping)
{
    public static SpatialBackendMaterial Default => new(1, 2, 240, 1);

    public void Validate()
    {
        if (!float.IsFinite(Friction) || Friction < 0 || !float.IsFinite(RecoveryVelocity) || RecoveryVelocity < 0
            || !float.IsFinite(SpringFrequency) || SpringFrequency <= 0 || !float.IsFinite(Damping) || Damping < 0)
            throw new ArgumentException("Invalid spatial contact material.");
    }
}

internal sealed class SpatialBackendBodyDefinition
{
    public SpatialBackendMotion Motion { get; }
    public IReadOnlyList<SpatialBackendPart> Parts { get; }
    public float Mass { get; }
    public SpatialBackendMaterial Material { get; }
    public uint CollisionLayer { get; }
    public uint CollisionMask { get; }
    public uint CollisionLayerUnion { get; }
    public uint CollisionMaskUnion { get; }
    public bool LockRotation { get; }

    public SpatialBackendBodyDefinition(SpatialBackendMotion motion, IEnumerable<SpatialBackendPart> parts,
        float mass = 1, SpatialBackendMaterial? material = null, uint collisionLayer = 1, uint collisionMask = uint.MaxValue,
        bool lockRotation = false)
    {
        if (!Enum.IsDefined(motion))
            throw new ArgumentOutOfRangeException(nameof(motion));
        ArgumentNullException.ThrowIfNull(parts);
        if (!float.IsFinite(mass) || mass <= 0 || !float.IsFinite(1 / mass))
            throw new ArgumentOutOfRangeException(nameof(mass));
        var copy = parts.ToArray();
        if (copy.Length == 0)
            throw new ArgumentException("A body requires at least one convex part.");
        foreach (var part in copy)
        {
            ValidateShape(part.Shape);
            if (!part.LocalPose.IsValid || !float.IsFinite(part.MassWeight) || part.MassWeight <= 0
                || part.Entity is { } entity && !entity.IsValid()
                || part.Surface is { } surface && !surface.OwnerFrame.IsValid())
                throw new ArgumentException("Invalid spatial body part.");
        }
        Motion = motion;
        Parts = Array.AsReadOnly(copy);
        Mass = mass;
        Material = material ?? SpatialBackendMaterial.Default;
        Material.Validate();
        CollisionLayer = collisionLayer;
        CollisionMask = collisionMask;
        foreach (var part in copy)
        {
            CollisionLayerUnion |= part.CollisionLayer ?? collisionLayer;
            CollisionMaskUnion |= part.CollisionMask ?? collisionMask;
        }
        LockRotation = lockRotation;
    }

    internal static void ValidateShape(SpatialConvexShape shape)
    {
        switch (shape)
        {
            case SpatialBox box when Finite(box.Size) && box.Size.X > 0 && box.Size.Y > 0 && box.Size.Z > 0:
            case SpatialSphere sphere when float.IsFinite(sphere.Radius) && sphere.Radius > 0:
            case SpatialCapsule capsule when float.IsFinite(capsule.Radius) && capsule.Radius > 0
                && float.IsFinite(capsule.Length) && capsule.Length >= 0:
                return;
            default:
                throw new ArgumentException("A spatial convex shape must have finite valid dimensions.", nameof(shape));
        }
    }

    internal static bool Finite(Vector3 vector)
        => float.IsFinite(vector.X) && float.IsFinite(vector.Y) && float.IsFinite(vector.Z);
}

/// <summary>Pose and velocity at the canonical entity origin, not the backend's center of mass.</summary>
internal readonly record struct SpatialBackendState(RigidTransform3 Pose, Vector3 LinearVelocity, Vector3 AngularVelocity);
internal readonly record struct SpatialBackendHit(SpatialBackendBodyId Body, SpatialBackendPart Source,
    Vector3 Point, Vector3 Normal, float Distance);

/// <summary>Fraction of the requested motion; an initial overlap has no unique contact point/normal.</summary>
internal readonly record struct SpatialBackendCastHit(SpatialBackendBodyId Body, SpatialBackendPart Source,
    float Fraction, Vector3? Point, Vector3? Normal, int ChildIndex = 0);

internal readonly record struct SpatialBackendOverlap(SpatialBackendBodyId Body, SpatialBackendPart Source,
    IReadOnlyList<SpatialContactGeometry> Contacts, int ChildIndex = 0);

internal interface ISpatialPhysicsBackend : IDisposable
{
    SpatialBackendBodyId AddBody(SpatialBackendBodyDefinition definition, SpatialBackendState state);
    void RemoveBody(SpatialBackendBodyId id);
    SpatialBackendState GetState(SpatialBackendBodyId id);
    void SetState(SpatialBackendBodyId id, SpatialBackendState state);
    void SetPaused(SpatialBackendBodyId id, bool paused);
    void ApplyImpulse(SpatialBackendBodyId id, Vector3 worldImpulse, Vector3 worldPoint);
    SpatialBackendConstraintId AddConstraint(SpatialBackendConstraintDefinition definition);
    void RemoveConstraint(SpatialBackendConstraintId id);
    IReadOnlyList<SpatialBackendContactEvent> DrainContactEvents();
    void Step(float fixedDelta);
    SpatialBackendHit? RayCast(Vector3 origin, Vector3 unitDirection, float distance, uint mask,
        SpatialBackendBodyId? ignoredBody = null, Func<SpatialBackendPart, bool>? filter = null);
    IReadOnlyList<SpatialBackendCastHit> ShapeCast(SpatialConvexShape shape, RigidTransform3 pose,
        Vector3 translation, Vector3 angularDisplacement, uint mask, SpatialBackendBodyId? ignoredBody = null);
    IReadOnlyList<SpatialBackendCastHit> Overlap(SpatialConvexShape shape, RigidTransform3 pose,
        uint mask, SpatialBackendBodyId? ignoredBody = null);
    IReadOnlyList<SpatialBackendOverlap> OverlapContacts(SpatialConvexShape shape, RigidTransform3 pose,
        uint mask, SpatialBackendBodyId? ignoredBody = null);
}
