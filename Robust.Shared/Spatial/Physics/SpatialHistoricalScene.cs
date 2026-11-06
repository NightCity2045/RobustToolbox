using System;
using System.Collections.Generic;
using System.Diagnostics.CodeAnalysis;
using System.Numerics;
using Robust.Shared.GameObjects;
using Robust.Shared.Maths;
using Robust.Shared.Timing;

namespace Robust.Shared.Spatial.Physics;

public readonly record struct SpatialHistoricalBodyState(RigidTransform3 Pose, Vector3 LinearVelocity,
    Vector3 AngularVelocity, ulong DefinitionRevision);

/// <summary>
/// Exact physical-tick snapshot. It expires when the configured window/budget
/// evicts it or its world is removed. Queries return world-space witnesses and
/// historical identities; content decides whether those identities are still
/// valid gameplay targets and how a missing tick affects the requested action.
/// </summary>
public sealed class SpatialHistoricalScene : ISpatialQueryService
{
    private readonly SpatialHistorySystem _owner;
    internal SpatialHistoryBody[] Bodies;
    internal readonly Dictionary<EntityUid, RigidTransform3> Poses;
    internal readonly Dictionary<EntityUid, SpatialGeometrySnapshot> Geometries;
    internal readonly int ShapeCount;
    public EntityUid World { get; }
    public GameTick Tick { get; }
    public bool IsAvailable { get; private set; } = true;

    internal SpatialHistoricalScene(SpatialHistorySystem owner, EntityUid world, GameTick tick,
        SpatialHistoryBody[] bodies, Dictionary<EntityUid, RigidTransform3> poses,
        Dictionary<EntityUid, SpatialGeometrySnapshot> geometries, int shapeCount)
    {
        _owner = owner;
        World = world;
        Tick = tick;
        Bodies = bodies;
        Poses = poses;
        Geometries = geometries;
        ShapeCount = shapeCount;
    }

    public bool TryGetPose(EntityUid entity, out RigidTransform3 pose)
        => Poses.TryGetValue(entity, out pose);

    public bool TryGetGeometry(EntityUid frame, [NotNullWhen(true)] out SpatialGeometrySnapshot? snapshot)
        => Geometries.TryGetValue(frame, out snapshot);

    public bool TryGetBodyState(EntityUid entity, out SpatialHistoricalBodyState state)
    {
        foreach (var body in Bodies)
        {
            if (body.Entity != entity) continue;
            state = new SpatialHistoricalBodyState(body.State.Pose, body.State.LinearVelocity, body.State.AngularVelocity,
                body.DefinitionRevision);
            return true;
        }
        state = default;
        return false;
    }

    internal void Expire()
    {
        IsAvailable = false;
        Bodies = Array.Empty<SpatialHistoryBody>();
        Poses.Clear();
        Geometries.Clear();
    }

    public SpatialHit? RayCast(SpatialQueryContext context, Vector3 origin, Vector3 unitDirection, float distance)
        => RayCast(context, origin, unitDirection, distance, false);

    public SpatialHit? FindSurface(SpatialQueryContext context, Vector3 origin, Vector3 unitDirection, float distance)
        => RayCast(context, origin, unitDirection, distance, true);

    private SpatialHit? RayCast(SpatialQueryContext context, Vector3 origin, Vector3 unitDirection, float distance,
        bool surfacesOnly)
    {
        var frame = Validate(context);
        var backend = _owner.BeginQuery(this);
        try
        {
            var hit = backend.RayCast(frame.TransformPoint(origin), frame.TransformVector(unitDirection), distance,
                context.Mask, filter: part => (!surfacesOnly || part.Surface != null) && Allow(context, part));
            return hit is { } value ? new SpatialHit(value.Source.Entity, value.Source.Surface, value.Point, value.Normal,
                value.Distance, value.Distance / distance, value.Source.MaterialProfile, value.Source.Revision) : null;
        }
        finally { _owner.EndQuery(); }
    }

    public IReadOnlyList<SpatialHit> ShapeCast(SpatialQueryContext context, SpatialShape shape, RigidTransform3 pose,
        Vector3 translation, Vector3 angularDisplacement)
    {
        ArgumentNullException.ThrowIfNull(shape);
        shape.Validate();
        var frame = Validate(context);
        var backend = _owner.BeginQuery(this);
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
        finally { _owner.EndQuery(); }
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
        var backend = _owner.BeginQuery(this);
        try
        {
            var result = new List<SpatialOverlap>();
            foreach (var hit in backend.OverlapContacts(shape.ToBackend(), frame.Compose(pose), context.Mask))
                if (Allow(context, hit.Source))
                    result.Add(new SpatialOverlap(new SpatialQueryCandidate(hit.Source.Entity, hit.Source.Surface,
                        hit.Source.MaterialProfile, hit.Source.Revision), hit.Contacts));
            return result.AsReadOnly();
        }
        finally { _owner.EndQuery(); }
    }

    private RigidTransform3 Validate(SpatialQueryContext context)
    {
        ArgumentNullException.ThrowIfNull(context);
        if (!IsAvailable) throw new InvalidOperationException("The historical scene has expired.");
        if (context.World != World || !Poses.TryGetValue(context.Frame, out var frame))
            throw new InvalidOperationException("The requested world/frame was not captured in this historical scene.");
        if (context.IgnoredEntity is { } ignored && !Poses.ContainsKey(ignored))
            throw new InvalidOperationException("The ignored entity was not captured in this historical scene.");
        foreach (var (uid, revision) in context.ExpectedGeometryRevisions)
            if (!Geometries.TryGetValue(uid, out var geometry) || geometry.Revision != revision)
                throw new InvalidOperationException("The requested geometry revision differs from the historical scene.");
        return frame;
    }

    private static bool Allow(SpatialQueryContext context, SpatialBackendPart part)
        => (context.IgnoredEntity == null || context.IgnoredEntity != part.Entity)
            && (context.Filter?.Allow(context, new SpatialQueryCandidate(part.Entity, part.Surface,
                part.MaterialProfile, part.Revision)) ?? true);
}
