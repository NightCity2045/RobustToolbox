using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using Robust.Shared.Maths;
using Robust.Shared.Serialization;
using Robust.Shared.Serialization.Manager.Attributes;
using Robust.Shared.Serialization.TypeSerializers.Implementations.Custom;

namespace Robust.Shared.Spatial.Physics;

public enum SpatialMotionOwner : byte
{
    Static,
    DynamicBody,
    Motor,
    AnchoredCompoundChild
}

[Serializable, NetSerializable, ImplicitDataDefinitionForInheritors]
public abstract partial class SpatialShape : ISerializationHooks
{
    public abstract void Validate();
    internal abstract SpatialConvexShape ToBackend();
    internal abstract SpatialShape Copy();
    void ISerializationHooks.AfterDeserialization() => Validate();
}

[Serializable, NetSerializable]
public sealed partial class SpatialBoxShape : SpatialShape
{
    [DataField("size", required: true)] public Vector3 Size { get; private set; }
    private SpatialBoxShape() { }
    public SpatialBoxShape(Vector3 size) { Size = size; Validate(); }
    public override void Validate() => SpatialBackendBodyDefinition.ValidateShape(ToBackend());
    internal override SpatialConvexShape ToBackend() => new SpatialBox(Size);
    internal override SpatialShape Copy() => new SpatialBoxShape(Size);
}

[Serializable, NetSerializable]
public sealed partial class SpatialSphereShape : SpatialShape
{
    [DataField("radius", required: true)] public float Radius { get; private set; }
    private SpatialSphereShape() { }
    public SpatialSphereShape(float radius) { Radius = radius; Validate(); }
    public override void Validate() => SpatialBackendBodyDefinition.ValidateShape(ToBackend());
    internal override SpatialConvexShape ToBackend() => new SpatialSphere(Radius);
    internal override SpatialShape Copy() => new SpatialSphereShape(Radius);
}

/// <summary>A capsule along local +Z; length excludes the hemispheres.</summary>
[Serializable, NetSerializable]
public sealed partial class SpatialCapsuleShape : SpatialShape
{
    [DataField("radius", required: true)] public float Radius { get; private set; }
    [DataField("length", required: true)] public float Length { get; private set; }
    private SpatialCapsuleShape() { }
    public SpatialCapsuleShape(float radius, float length) { Radius = radius; Length = length; Validate(); }
    public override void Validate() => SpatialBackendBodyDefinition.ValidateShape(ToBackend());
    internal override SpatialConvexShape ToBackend() => new SpatialCapsule(Radius, Length);
    internal override SpatialShape Copy() => new SpatialCapsuleShape(Radius, Length);
}

/// <summary>Collider offset within the existing entity transform, not an entity position.</summary>
[Serializable, NetSerializable, DataDefinition]
public sealed partial class SpatialColliderDefinition : ISerializationHooks
{
    [DataField("shape", required: true)] public SpatialShape Shape { get; private set; } = default!;
    [DataField("offset")] public Vector3 Offset { get; private set; }
    [DataField("rotation", customTypeSerializer: typeof(TransformRotationSerializer))]
    public Quaternion Rotation { get; private set; } = Quaternion.Identity;
    [DataField("massWeight")] public float MassWeight { get; private set; } = 1;
    public RigidTransform3 LocalPose => new(Offset, Rotation);
    private SpatialColliderDefinition() { }

    public SpatialColliderDefinition(SpatialShape shape, RigidTransform3 localPose, float massWeight = 1)
    {
        ArgumentNullException.ThrowIfNull(shape);
        if (!localPose.IsValid)
            throw new ArgumentException("A collider requires a valid local rigid pose.");
        Shape = shape.Copy();
        Offset = localPose.Position;
        Rotation = localPose.Rotation;
        MassWeight = massWeight;
        Validate();
    }

    public void Validate()
    {
        ArgumentNullException.ThrowIfNull(Shape);
        Shape.Validate();
        _ = LocalPose;
        if (!float.IsFinite(MassWeight) || MassWeight <= 0)
            throw new ArgumentOutOfRangeException(nameof(MassWeight));
    }
    internal SpatialColliderDefinition Copy() => new(Shape, LocalPose, MassWeight);
    void ISerializationHooks.AfterDeserialization() => Validate();
}

[Serializable, NetSerializable, DataRecord]
public readonly partial record struct SpatialContactMaterial(float Friction, float RecoveryVelocity, float SpringFrequency, float Damping)
    : ISerializationHooks
{
    public static SpatialContactMaterial Default => new(1, 2, 240, 1);
    public void Validate() => ToBackend().Validate();
    internal SpatialBackendMaterial ToBackend() => new(Friction, RecoveryVelocity, SpringFrequency, Damping);
    void ISerializationHooks.AfterDeserialization() => Validate();
}

/// <summary>
/// Neutral mass, shape and collision settings. Canonical frame geometry can supply
/// the collider parts instead of explicit shapes. No backend indices or entity pose
/// are included, and gameplay profiles remain in content.
/// </summary>
[Serializable, NetSerializable, DataDefinition]
public sealed partial class SpatialBodyDefinition : ISerializationHooks
{
    [DataField("owner")] public SpatialMotionOwner Owner { get; private set; } = SpatialMotionOwner.DynamicBody;
    [DataField("mass")] public float Mass { get; private set; } = 1;
    [DataField("material")] public SpatialContactMaterial Material { get; private set; } = SpatialContactMaterial.Default;
    [DataField("collisionLayer")] public uint CollisionLayer { get; private set; } = 1;
    [DataField("collisionMask")] public uint CollisionMask { get; private set; } = uint.MaxValue;
    [DataField("geometry")] public bool UseGeometry { get; private set; }
    [DataField("colliders")] private SpatialColliderDefinition[] _colliders = Array.Empty<SpatialColliderDefinition>();
    public IReadOnlyList<SpatialColliderDefinition> Colliders => Array.AsReadOnly(_colliders);
    private SpatialBodyDefinition() { }

    public SpatialBodyDefinition(SpatialMotionOwner owner, IEnumerable<SpatialColliderDefinition>? colliders = null,
        bool useGeometry = false, float mass = 1, SpatialContactMaterial? material = null,
        uint collisionLayer = 1, uint collisionMask = uint.MaxValue)
    {
        Owner = owner;
        UseGeometry = useGeometry;
        Mass = mass;
        Material = material ?? SpatialContactMaterial.Default;
        CollisionLayer = collisionLayer;
        CollisionMask = collisionMask;
        _colliders = colliders?.Select(c => c.Copy()).ToArray() ?? Array.Empty<SpatialColliderDefinition>();
        Validate();
    }

    public void Validate()
    {
        if (!Enum.IsDefined(Owner) || !float.IsFinite(Mass) || Mass <= 0 || !float.IsFinite(1 / Mass))
            throw new ArgumentException("Invalid spatial motion owner or mass.");
        Material.Validate();
        if (_colliders == null || UseGeometry && _colliders.Length != 0 || !UseGeometry && _colliders.Length == 0)
            throw new ArgumentException("Choose canonical geometry or a nonempty explicit collider set.");
        foreach (var collider in _colliders)
        {
            ArgumentNullException.ThrowIfNull(collider);
            collider.Validate();
        }
    }
    internal SpatialBodyDefinition Copy() => new(Owner, _colliders, UseGeometry, Mass, Material, CollisionLayer, CollisionMask);
    void ISerializationHooks.AfterDeserialization() => Validate();
}

[Serializable, NetSerializable, DataRecord]
public readonly partial record struct SpatialPhysicsSettings(Vector3 Gravity, int Substeps, int SolverIterations) : ISerializationHooks
{
    public static SpatialPhysicsSettings Default => new(new Vector3(0, 0, -9.81f), 1, 8);
    public void Validate()
    {
        if (!SpatialBackendBodyDefinition.Finite(Gravity) || Substeps <= 0 || SolverIterations <= 0)
            throw new ArgumentException("Invalid spatial physics world settings.");
    }
    void ISerializationHooks.AfterDeserialization() => Validate();
}
