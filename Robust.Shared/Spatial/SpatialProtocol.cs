using System;
using System.Globalization;
using System.Linq;
using System.Security.Cryptography;
using System.Text;
using System.Text.Json;
using BepuPhysics;
using Lidgren.Network;
using Robust.Shared.EntitySerialization;

namespace Robust.Shared.Spatial;

/// <summary>Fork wire identity, independent of the upstream engine version advertised by a launcher.</summary>
public static class SpatialProtocol
{
    private const uint Magic = 0x53503134;
    private const int HeaderBits = (4 + 4 + 32) * 8;
    private static readonly JsonDocument Description = LoadDescription();
    public static string Id => Description.RootElement.GetProperty("id").GetString()!;
    public static uint Revision => Description.RootElement.GetProperty("wire_revision").GetUInt32();
    public static string Fingerprint { get; } = HashDescription(Description.RootElement);
    internal static SpatialProtocolStamp Current => new(Revision, Fingerprint);

    private static JsonDocument LoadDescription()
    {
        using var stream = typeof(SpatialProtocol).Assembly.GetManifestResourceStream("Robust.SpatialProtocol")
            ?? throw new InvalidOperationException("Spatial protocol descriptor is missing.");
        var document = JsonDocument.Parse(stream);
        var root = document.RootElement;
        if (root.GetProperty("map_format").GetInt32() != EntitySerializer.MapFormatVersion
            || root.GetProperty("transform_schema").GetInt32() != EntitySerializer.TransformSchemaVersion
            || root.GetProperty("world_schema").GetInt32() != SpatialWorldComponent.CurrentDefinitionSchema
            || root.GetProperty("geometry_schema").GetInt32() != SpatialGeometryComponent.CurrentSchema
            || root.GetProperty("geometry_wire_schema").GetUInt32() != Networking.SpatialGeometryWireCodec.Version
            || root.GetProperty("physics_backend").GetString() != typeof(Simulation).Assembly.GetName().Name
            || root.GetProperty("physics_version").GetString() != typeof(Simulation).Assembly.GetName().Version!.ToString(3))
            throw new InvalidOperationException("Spatial protocol descriptor does not match the compiled schemas/backend.");
        return document;
    }

    internal static string HashDescription(JsonElement root)
    {
        // Stable across checkout line endings, JSON indentation and property order.
        var canonical = string.Concat(root.EnumerateObject().OrderBy(p => p.Name, StringComparer.Ordinal)
            .Select(p => p.Name + "=" + p.Value.ToString() + "\n"));
        return Convert.ToHexString(SHA256.HashData(Encoding.UTF8.GetBytes(canonical)));
    }

    internal static void Write(NetBuffer buffer, SpatialProtocolStamp stamp)
    {
        buffer.WritePadBits();
        buffer.Write(Magic);
        buffer.Write(stamp.Revision);
        buffer.Write(Convert.FromHexString(stamp.Fingerprint!));
    }

    internal static SpatialProtocolStamp Read(NetBuffer buffer)
    {
        var aligned = (buffer.Position + 7) & ~7L;
        if (buffer.LengthBits - aligned < HeaderBits)
            return default;
        buffer.ReadPadBits();
        if (buffer.ReadUInt32() != Magic)
            return default;
        return new SpatialProtocolStamp(buffer.ReadUInt32(), Convert.ToHexString(buffer.ReadBytes(32)));
    }

    internal static string? Rejection(SpatialProtocolStamp remote)
    {
        if (remote.Fingerprint == null)
            return $"This connection requires {Id} protocol {Revision.ToString(CultureInfo.InvariantCulture)}; ordinary Robust 295 is incompatible.";
        if (remote.Revision != Revision)
            return $"Incompatible {Id} protocol revision: expected {Revision}, received {remote.Revision}. Install the matching client/server pair.";
        if (remote.Fingerprint != Fingerprint)
            return $"Incompatible {Id} schema/physics fingerprint. Install the matching client/server pair.";
        return null;
    }
}

internal readonly record struct SpatialProtocolStamp(uint Revision, string? Fingerprint);
