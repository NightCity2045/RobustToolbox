using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.Numerics;
using Robust.Shared.GameObjects;
using Robust.Shared.Maths;

namespace Robust.Shared.Spatial.Physics;

/// <summary>
/// Explicit world and input coordinate frame. Purpose/filter are supplied by the
/// caller; the engine does not equate collision, interaction and gas permeability.
/// Queries see the world's active collision scene, excluding contained bodies.
/// Container access remains a content policy, not an implicit query into storage.
/// </summary>
public sealed class SpatialQueryContext
{
    public EntityUid World { get; }
    public EntityUid Frame { get; }
    public string Purpose { get; }
    public uint Mask { get; }
    public EntityUid? IgnoredEntity { get; }
    public ISpatialQueryFilter? Filter { get; }
    public IReadOnlyDictionary<EntityUid, ulong> ExpectedGeometryRevisions { get; }

    public SpatialQueryContext(EntityUid world, EntityUid frame, string purpose, uint mask,
        EntityUid? ignoredEntity = null, ISpatialQueryFilter? filter = null,
        IReadOnlyDictionary<EntityUid, ulong>? expectedGeometryRevisions = null)
    {
        if (!world.IsValid() || !frame.IsValid() || string.IsNullOrWhiteSpace(purpose)
            || ignoredEntity is { } ignored && !ignored.IsValid())
            throw new ArgumentException("Queries require valid world/frame references and an explicit purpose.");
        World = world;
        Frame = frame;
        Purpose = purpose;
        Mask = mask;
        IgnoredEntity = ignoredEntity;
        Filter = filter;
        ExpectedGeometryRevisions = new ReadOnlyDictionary<EntityUid, ulong>(expectedGeometryRevisions == null
            ? new Dictionary<EntityUid, ulong>() : new Dictionary<EntityUid, ulong>(expectedGeometryRevisions));
    }
}

public readonly record struct SpatialQueryCandidate(EntityUid? Entity, SpatialSurfaceReference? Surface,
    string MaterialProfile, ulong Revision);

/// <summary>A read-only policy. Implementations must not mutate entities or query recursively.</summary>
public interface ISpatialQueryFilter
{
    bool Allow(SpatialQueryContext context, SpatialQueryCandidate candidate);
}

/// <summary>
/// World-space witness and source identity. For initial overlap there is no unique
/// point/normal: both are null and Fraction/Distance are zero. For a rotating cast,
/// Distance measures center translation; Fraction identifies progress of the full motion.
/// </summary>
public readonly record struct SpatialHit(EntityUid? Entity, SpatialSurfaceReference? Surface,
    Vector3? Point, Vector3? Normal, float Distance, float Fraction, string MaterialProfile, ulong Revision)
{
    public bool InitialOverlap => Point == null;
}

public interface ISpatialQueryService
{
    SpatialHit? RayCast(SpatialQueryContext context, Vector3 origin, Vector3 unitDirection, float distance);
    IReadOnlyList<SpatialHit> ShapeCast(SpatialQueryContext context, SpatialShape shape, RigidTransform3 pose,
        Vector3 translation, Vector3 angularDisplacement);
    IReadOnlyList<SpatialHit> Overlap(SpatialQueryContext context, SpatialShape shape, RigidTransform3 pose);
    bool IsVolumeFree(SpatialQueryContext context, SpatialShape shape, RigidTransform3 pose);
    SpatialHit? FindSurface(SpatialQueryContext context, Vector3 origin, Vector3 unitDirection, float distance);
}
