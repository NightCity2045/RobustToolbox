using System;
using System.Collections.Generic;
using System.Numerics;
using Robust.Shared.GameObjects;
using Robust.Shared.Maths;

namespace Robust.Shared.Spatial.Physics;

/// <summary>Shared query implementation for immutable pose/geometry snapshots.</summary>
internal sealed class SpatialSnapshotQueries : ISpatialQueryService
{
    private readonly EntityUid World;
    private readonly IReadOnlyDictionary<EntityUid, RigidTransform3> Poses;
    private readonly IReadOnlyDictionary<EntityUid, SpatialGeometrySnapshot> Geometries;
    private readonly Func<bool> _available;
    private readonly Func<ISpatialPhysicsBackend> _begin;
    private readonly Action _end;
    private readonly Box3? _coverage;

    internal SpatialSnapshotQueries(EntityUid world, IReadOnlyDictionary<EntityUid, RigidTransform3> poses,
        IReadOnlyDictionary<EntityUid, SpatialGeometrySnapshot> geometries, Func<bool> available,
        Func<ISpatialPhysicsBackend> begin, Action end, Box3? coverage = null)
    {
        World = world; Poses = poses; Geometries = geometries;
        _available = available; _begin = begin; _end = end; _coverage = coverage;
    }

    public SpatialHit? RayCast(SpatialQueryContext context, Vector3 origin, Vector3 unitDirection, float distance)
        => RayCast(context, origin, unitDirection, distance, false);

    public SpatialHit? FindSurface(SpatialQueryContext context, Vector3 origin, Vector3 unitDirection, float distance)
        => RayCast(context, origin, unitDirection, distance, true);

    private SpatialHit? RayCast(SpatialQueryContext context, Vector3 origin, Vector3 unitDirection, float distance,
        bool surfacesOnly)
    {
        var frame = Validate(context);
        CheckCoverage(frame.TransformPoint(origin), frame.TransformPoint(origin + unitDirection * distance), 0);
        var backend = _begin();
        try
        {
            var hit = backend.RayCast(frame.TransformPoint(origin), frame.TransformVector(unitDirection), distance,
                context.Mask, filter: part => (!surfacesOnly || part.Surface != null) && Allow(context, part));
            return hit is { } value ? new SpatialHit(value.Source.Entity, value.Source.Surface, value.Point, value.Normal,
                value.Distance, value.Distance / distance, value.Source.MaterialProfile, value.Source.Revision) : null;
        }
        finally { _end(); }
    }

    public IReadOnlyList<SpatialHit> ShapeCast(SpatialQueryContext context, SpatialShape shape, RigidTransform3 pose,
        Vector3 translation, Vector3 angularDisplacement)
    {
        ArgumentNullException.ThrowIfNull(shape);
        shape.Validate();
        var frame = Validate(context);
        var center = frame.TransformPoint(pose.Position);
        CheckCoverage(center, center + frame.TransformVector(translation), Radius(shape));
        var backend = _begin();
        try
        {
            var hits = backend.ShapeCast(shape.ToBackend(), frame.Compose(pose), frame.TransformVector(translation),
                frame.TransformVector(angularDisplacement), context.Mask);
            var result = new List<SpatialHit>();
            foreach (var hit in hits)
                if (Allow(context, hit.Source))
                    result.Add(new SpatialHit(hit.Source.Entity, hit.Source.Surface, hit.Point, hit.Normal,
                        hit.Fraction * translation.Length(), hit.Fraction, hit.Source.MaterialProfile, hit.Source.Revision));
            return result.AsReadOnly();
        }
        finally { _end(); }
    }

    public IReadOnlyList<SpatialHit> Overlap(SpatialQueryContext context, SpatialShape shape, RigidTransform3 pose)
        => ShapeCast(context, shape, pose, Vector3.Zero, Vector3.Zero);

    public bool IsVolumeFree(SpatialQueryContext context, SpatialShape shape, RigidTransform3 pose)
        => Overlap(context, shape, pose).Count == 0;

    public IReadOnlyList<SpatialOverlap> OverlapContacts(SpatialQueryContext context, SpatialShape shape, RigidTransform3 pose)
    {
        ArgumentNullException.ThrowIfNull(shape);
        shape.Validate();
        var frame = Validate(context);
        var center = frame.TransformPoint(pose.Position);
        CheckCoverage(center, center, Radius(shape));
        var backend = _begin();
        try
        {
            var result = new List<SpatialOverlap>();
            foreach (var hit in backend.OverlapContacts(shape.ToBackend(), frame.Compose(pose), context.Mask))
                if (Allow(context, hit.Source))
                    result.Add(new SpatialOverlap(new SpatialQueryCandidate(hit.Source.Entity, hit.Source.Surface,
                        hit.Source.MaterialProfile, hit.Source.Revision), hit.Contacts));
            return result.AsReadOnly();
        }
        finally { _end(); }
    }

    private static float Radius(SpatialShape shape) => shape switch
    {
        SpatialBoxShape box => box.Size.Length() * 0.5f,
        SpatialSphereShape sphere => sphere.Radius,
        SpatialCapsuleShape capsule => capsule.Radius + capsule.Length * 0.5f,
        _ => throw new ArgumentException("Unknown spatial shape."),
    };

    private void CheckCoverage(Vector3 start, Vector3 end, float radius)
    {
        if (_coverage is not { } bounds) return;
        // A sphere encloses every orientation throughout a rotational sweep.
        // Unknown volume is never interpreted as an empty collision scene.
        if (!float.IsFinite(radius) || radius < 0
            || !bounds.Contains(Vector3.Min(start, end) - new Vector3(radius))
            || !bounds.Contains(Vector3.Max(start, end) + new Vector3(radius)))
            throw new InvalidOperationException("The query extends beyond certified geometry coverage.");
    }

    private RigidTransform3 Validate(SpatialQueryContext context)
    {
        ArgumentNullException.ThrowIfNull(context);
        if (!_available()) throw new InvalidOperationException("The query snapshot is unavailable.");
        if (context.World != World || !Poses.TryGetValue(context.Frame, out var frame))
            throw new InvalidOperationException("The requested world/frame was not captured in this query snapshot.");
        if (context.IgnoredEntity is { } ignored && !Poses.ContainsKey(ignored))
            throw new InvalidOperationException("The ignored entity was not captured in this query snapshot.");
        foreach (var (uid, revision) in context.ExpectedGeometryRevisions)
            if (!Geometries.TryGetValue(uid, out var geometry) || geometry.Revision != revision)
                throw new InvalidOperationException("The requested geometry revision differs from the query snapshot.");
        return frame;
    }

    private static bool Allow(SpatialQueryContext context, SpatialBackendPart part)
        => (context.IgnoredEntity == null || context.IgnoredEntity != part.Entity)
            && (context.Filter?.Allow(context, new SpatialQueryCandidate(part.Entity, part.Surface,
                part.MaterialProfile, part.Revision)) ?? true);
}
