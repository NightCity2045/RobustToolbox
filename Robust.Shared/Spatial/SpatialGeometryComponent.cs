using System;
using System.Collections.Generic;
using Robust.Shared.GameObjects;
using Robust.Shared.Serialization.Manager.Attributes;

namespace Robust.Shared.Spatial;

[ByRefEvent]
internal readonly struct SpatialGeometryStopping;

/// <summary>
/// Canonical semantic geometry of one transform frame. Derived meshes, backend
/// handles and spatial indexes are not serialized here. Geometry streaming is
/// separate from ordinary component replication.
/// </summary>
[RegisterComponent]
public sealed partial class SpatialGeometryComponent : Component
{
    // Ephemeral capture identity: removing/readding a component cannot make an
    // old background result current merely by reusing its numerical revision.
    internal readonly object CaptureIdentity = new();
    // A new component/load gets a new wire lifetime, even if revision returns to zero.
    // This transport token is not persisted authoring state.
    internal readonly Guid WireGeneration = Guid.NewGuid();
    public const uint CurrentSchema = 1;
    [DataField("schema")] internal uint _schema = CurrentSchema;
    [DataField("revision")] internal ulong _revision;
    [DataField("boundaries")] internal List<BoundarySegment> _boundaries = new();
    [DataField("slabs")] internal List<SlabSurface> _slabs = new();
    [DataField("tileChunks")] internal List<SpatialTileChunk> _tileChunks = new();

    public uint Schema => _schema;
    public ulong Revision => _revision;
}
