using System;
using System.Globalization;
using System.Numerics;
using Robust.Shared.IoC;
using Robust.Shared.Maths;
using Robust.Shared.Serialization.Manager;
using Robust.Shared.Serialization.Markdown;
using Robust.Shared.Serialization.Markdown.Validation;
using Robust.Shared.Serialization.Markdown.Value;
using Robust.Shared.Serialization.TypeSerializers.Interfaces;
using Robust.Shared.Utility;

namespace Robust.Shared.Serialization.TypeSerializers.Implementations.Custom;

/// <summary>
/// Reads both legacy XY and canonical XYZ transform positions. Writes XYZ only.
/// This field serializer does not decide the enclosing map's schema or migration policy.
/// </summary>
public sealed class TransformPositionSerializer : ITypeSerializer<Vector3, ValueDataNode>, ITypeCopyCreator<Vector3>
{
    public Vector3 Read(ISerializationManager serializationManager, ValueDataNode node,
        IDependencyCollection dependencies, SerializationHookContext hookCtx,
        ISerializationContext? context = null,
        ISerializationManager.InstantiationDelegate<Vector3>? instanceProvider = null)
    {
        if (!TryRead(node.Value, out var position))
            throw new InvalidMappingException("Transform position requires two or three finite coordinates.");
        return position;
    }

    public ValidationNode Validate(ISerializationManager serializationManager, ValueDataNode node,
        IDependencyCollection dependencies, ISerializationContext? context = null)
        => TryRead(node.Value, out _) ? new ValidatedValueNode(node)
            : new ErrorNode(node, "Transform position requires two or three finite coordinates.");

    public DataNode Write(ISerializationManager serializationManager, Vector3 value,
        IDependencyCollection dependencies, bool alwaysWrite = false, ISerializationContext? context = null)
    {
        _ = new RigidTransform3(value, Quaternion.Identity);
        return new ValueDataNode(FormattableString.Invariant($"{value.X:R},{value.Y:R},{value.Z:R}"));
    }

    public Vector3 CreateCopy(ISerializationManager serializationManager, Vector3 source,
        IDependencyCollection dependencies, SerializationHookContext hookCtx, ISerializationContext? context = null)
        => source;

    private static bool TryRead(string text, out Vector3 position)
    {
        position = default;
        if (!VectorSerializerUtility.TryParseArgs(text, 3, out var parts)
            && !VectorSerializerUtility.TryParseArgs(text, 2, out parts))
            return false;

        if (!TryFinite(parts[0], out var x) || !TryFinite(parts[1], out var y))
            return false;
        var z = 0f;
        if (parts.Length == 3 && !TryFinite(parts[2], out z))
            return false;
        position = new Vector3(x, y, z);
        return true;
    }

    internal static bool TryFinite(string text, out float value)
        => float.TryParse(text, NumberStyles.Float, CultureInfo.InvariantCulture, out value) && float.IsFinite(value);
}

/// <summary>
/// Reads legacy degree/radian yaw or an XYZW quaternion. Writes a normalized,
/// canonical XYZW quaternion, preserving pitch and roll.
/// </summary>
public sealed class TransformRotationSerializer : ITypeSerializer<Quaternion, ValueDataNode>, ITypeCopyCreator<Quaternion>
{
    public Quaternion Read(ISerializationManager serializationManager, ValueDataNode node,
        IDependencyCollection dependencies, SerializationHookContext hookCtx,
        ISerializationContext? context = null,
        ISerializationManager.InstantiationDelegate<Quaternion>? instanceProvider = null)
    {
        if (!TryRead(node.Value, out var rotation))
            throw new InvalidMappingException("Transform rotation requires a finite angle or nonzero finite XYZW quaternion.");
        return rotation;
    }

    public ValidationNode Validate(ISerializationManager serializationManager, ValueDataNode node,
        IDependencyCollection dependencies, ISerializationContext? context = null)
        => TryRead(node.Value, out _) ? new ValidatedValueNode(node)
            : new ErrorNode(node, "Transform rotation requires a finite angle or nonzero finite XYZW quaternion.");

    public DataNode Write(ISerializationManager serializationManager, Quaternion value,
        IDependencyCollection dependencies, bool alwaysWrite = false, ISerializationContext? context = null)
    {
        value = new RigidTransform3(Vector3.Zero, value).Rotation;
        return new ValueDataNode(FormattableString.Invariant($"{value.X:R},{value.Y:R},{value.Z:R},{value.W:R}"));
    }

    public Quaternion CreateCopy(ISerializationManager serializationManager, Quaternion source,
        IDependencyCollection dependencies, SerializationHookContext hookCtx, ISerializationContext? context = null)
        => source;

    private static bool TryRead(string text, out Quaternion rotation)
    {
        rotation = Quaternion.Identity;
        if (VectorSerializerUtility.TryParseArgs(text, 4, out var parts))
        {
            if (!TransformPositionSerializer.TryFinite(parts[0], out var x)
                || !TransformPositionSerializer.TryFinite(parts[1], out var y)
                || !TransformPositionSerializer.TryFinite(parts[2], out var z)
                || !TransformPositionSerializer.TryFinite(parts[3], out var w)
                || x == 0 && y == 0 && z == 0 && w == 0)
                return false;
            rotation = new RigidTransform3(Vector3.Zero, new Quaternion(x, y, z, w)).Rotation;
            return true;
        }

        var radians = text.EndsWith("rad", StringComparison.Ordinal);
        var angleText = radians ? text[..^3] : text;
        if (!double.TryParse(angleText, NumberStyles.Float, CultureInfo.InvariantCulture, out var angle)
            || !double.IsFinite(angle))
            return false;

        // Reduce before converting degrees to radians or narrowing to float.
        // Even very large finite legacy angles must not overflow the quaternion.
        angle = radians ? angle % Math.Tau : angle % 360 * (Math.PI / 180);
        rotation = new RigidTransform3(Vector3.Zero,
            Quaternion.CreateFromAxisAngle(Vector3.UnitZ, (float) angle)).Rotation;
        return true;
    }
}
