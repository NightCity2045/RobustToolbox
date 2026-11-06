using System;
using System.Numerics;
using Robust.Shared.GameObjects;
using Robust.Shared.GameStates;
using Robust.Shared.Maths;
using Robust.Shared.Serialization;
using Robust.Shared.Serialization.Manager.Attributes;
using Robust.Shared.Serialization.TypeSerializers.Implementations.Custom;

namespace Robust.Shared.Spatial.Physics;

/// <summary>Canonical connection of entity origins; backend handles are derived and private.</summary>
[DataDefinition]
public sealed partial class SpatialConstraintDefinition : ISerializationHooks
{
    [DataField("bodyA", required: true)] public EntityUid BodyA { get; private set; }
    [DataField("bodyB", required: true)] public EntityUid BodyB { get; private set; }
    [DataField("kind")] public SpatialConstraintKind Kind { get; private set; }
    [DataField("anchorA")] public Vector3 LocalAnchorA { get; private set; }
    [DataField("anchorB")] public Vector3 LocalAnchorB { get; private set; }
    [DataField("relativeOffset")] public Vector3 RelativeOffset { get; private set; }
    [DataField("relativeRotation", customTypeSerializer: typeof(TransformRotationSerializer))]
    public Quaternion RelativeRotation { get; private set; } = Quaternion.Identity;
    [DataField("minimumDistance")] public float MinimumDistance { get; private set; }
    [DataField("maximumDistance")] public float MaximumDistance { get; private set; }
    [DataField("frequency")] public float SpringFrequency { get; private set; } = 30;
    [DataField("damping")] public float Damping { get; private set; } = 1;
    [DataField("collideConnected")] public bool CollideConnected { get; private set; }
    public RigidTransform3 RelativePose => new(RelativeOffset, RelativeRotation);
    private SpatialConstraintDefinition() { }

    public SpatialConstraintDefinition(EntityUid bodyA, EntityUid bodyB, SpatialConstraintKind kind,
        Vector3 localAnchorA = default, Vector3 localAnchorB = default, RigidTransform3? relativePose = null,
        float minimumDistance = 0, float maximumDistance = 0, float springFrequency = 30, float damping = 1,
        bool collideConnected = false)
    {
        BodyA = bodyA;
        BodyB = bodyB;
        Kind = kind;
        LocalAnchorA = localAnchorA;
        LocalAnchorB = localAnchorB;
        var relative = relativePose ?? RigidTransform3.Identity;
        RelativeOffset = relative.Position;
        RelativeRotation = relative.Rotation;
        MinimumDistance = minimumDistance;
        MaximumDistance = maximumDistance;
        SpringFrequency = springFrequency;
        Damping = damping;
        CollideConnected = collideConnected;
        Validate();
    }

    public void Validate()
    {
        if (!BodyA.IsValid() || !BodyB.IsValid() || BodyA == BodyB)
            throw new ArgumentException("A spatial connection requires two distinct entities.");
        SpatialBackendConstraintDefinition.ValidateSettings(Kind, LocalAnchorA, LocalAnchorB, RelativePose,
            MinimumDistance, MaximumDistance, SpringFrequency, Damping);
    }

    internal SpatialConstraintDefinition Copy() => new(BodyA, BodyB, Kind, LocalAnchorA, LocalAnchorB,
        RelativePose, MinimumDistance, MaximumDistance, SpringFrequency, Damping, CollideConnected);
    internal SpatialBackendConstraintDefinition ToBackend(SpatialBackendBodyId a, SpatialBackendBodyId b)
        => new(Kind, a, b, LocalAnchorA, LocalAnchorB, RelativePose, MinimumDistance, MaximumDistance,
            SpringFrequency, Damping, CollideConnected);
    void ISerializationHooks.AfterDeserialization() => Validate();
}

[RegisterComponent, NetworkedComponent]
public sealed partial class SpatialConstraintComponent : Component
{
    [DataField("definition", required: true)] internal SpatialConstraintDefinition _definition = default!;
    public bool HasDefinition => _definition != null;
    public SpatialConstraintDefinition Definition => _definition ?? throw new InvalidOperationException("Constraint definition is not available yet.");
}

/// <summary>Wire references use NetEntity; map references use the definition's EntityUid serializer.</summary>
[Serializable, NetSerializable]
public sealed class SpatialConstraintComponentState(NetEntity a, NetEntity b, SpatialConstraintKind kind,
    Vector3 anchorA, Vector3 anchorB, Vector3 relativeOffset, Quaternion relativeRotation,
    float minimum, float maximum, float frequency, float damping, bool collideConnected) : ComponentState
{
    public readonly NetEntity A = a;
    public readonly NetEntity B = b;
    public readonly SpatialConstraintKind Kind = kind;
    public readonly Vector3 AnchorA = anchorA;
    public readonly Vector3 AnchorB = anchorB;
    public readonly Vector3 RelativeOffset = relativeOffset;
    public readonly Quaternion RelativeRotation = relativeRotation;
    public readonly float Minimum = minimum;
    public readonly float Maximum = maximum;
    public readonly float Frequency = frequency;
    public readonly float Damping = damping;
    public readonly bool CollideConnected = collideConnected;
}
