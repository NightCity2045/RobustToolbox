using System;
using System.Collections.Generic;
using System.Numerics;
using Robust.Shared.GameObjects;
using Robust.Shared.IoC;
using Robust.Shared.Maths;

namespace Robust.Shared.Spatial.Physics;

/// <summary>World/frame-aware public queries over the trusted physical scene.</summary>
public sealed partial class SpatialQuerySystem : EntitySystem, ISpatialQueryService
{
    [Dependency] private SharedSpatialPhysicsSystem _physics = default!;
    [Dependency] private SharedSpatialSystem _spatial = default!;
    [Dependency] private SharedTransformSystem _transforms = default!;
    private bool _querying;

    public SpatialHit? RayCast(SpatialQueryContext context, Vector3 origin, Vector3 unitDirection, float distance)
        => RayCast(context, origin, unitDirection, distance, false);

    /// <summary>Nearest semantic surface along an explicit search direction.</summary>
    public SpatialHit? FindSurface(SpatialQueryContext context, Vector3 origin, Vector3 unitDirection, float distance)
        => RayCast(context, origin, unitDirection, distance, true);

    private SpatialHit? RayCast(SpatialQueryContext context, Vector3 origin, Vector3 unitDirection, float distance,
        bool surfacesOnly)
    {
        var (backend, frame) = ValidateContext(context);
        _querying = true;
        try
        {
            var hit = backend.RayCast(frame.TransformPoint(origin), frame.TransformVector(unitDirection), distance,
                context.Mask, filter: part => (!surfacesOnly || part.Surface != null) && Allow(context, part));
            if (hit is not { } value)
                return null;
            var source = value.Source;
            return new SpatialHit(source.Entity, source.Surface, value.Point, value.Normal,
                value.Distance, value.Distance / distance, source.MaterialProfile, source.Revision);
        }
        finally { _querying = false; }
    }

    public IReadOnlyList<SpatialHit> ShapeCast(SpatialQueryContext context, SpatialShape shape, RigidTransform3 pose,
        Vector3 translation, Vector3 angularDisplacement)
    {
        ArgumentNullException.ThrowIfNull(shape);
        shape.Validate();
        if (!pose.IsValid || !float.IsFinite(translation.LengthSquared()))
            throw new ArgumentException("Invalid cast pose or translation.");
        var (backend, frame) = ValidateContext(context);
        _querying = true;
        try
        {
            var hits = backend.ShapeCast(shape.ToBackend(), frame.Compose(pose), frame.TransformVector(translation),
                frame.TransformVector(angularDisplacement), context.Mask);
            var result = new List<SpatialHit>();
            var length = translation.Length();
            foreach (var hit in hits)
            {
                if (!Allow(context, hit.Source))
                    continue;
                var source = hit.Source;
                result.Add(new SpatialHit(source.Entity, source.Surface, hit.Point, hit.Normal,
                    hit.Fraction * length, hit.Fraction, source.MaterialProfile, source.Revision));
            }
            return result.AsReadOnly();
        }
        finally { _querying = false; }
    }

    public IReadOnlyList<SpatialHit> Overlap(SpatialQueryContext context, SpatialShape shape, RigidTransform3 pose)
        => ShapeCast(context, shape, pose, Vector3.Zero, Vector3.Zero);

    public bool IsVolumeFree(SpatialQueryContext context, SpatialShape shape, RigidTransform3 pose)
        => Overlap(context, shape, pose).Count == 0;

    private (ISpatialPhysicsBackend Backend, RigidTransform3 Frame) ValidateContext(SpatialQueryContext context)
    {
        ArgumentNullException.ThrowIfNull(context);
        if (_querying)
            throw new InvalidOperationException("A spatial query policy cannot recursively query the scene.");
        if (_spatial.GetWorld(context.Frame).Owner != context.World || _spatial.GetWorld(context.World).Owner != context.World)
            throw new InvalidOperationException("Query frame does not belong to the requested world.");
        if (context.IgnoredEntity is { } ignored && _spatial.GetWorld(ignored).Owner != context.World)
            throw new InvalidOperationException("Ignored entity belongs to another world.");
        foreach (var (frame, revision) in context.ExpectedGeometryRevisions)
        {
            if (_spatial.GetWorld(frame).Owner != context.World || !TryComp<SpatialGeometryComponent>(frame, out var geometry)
                || geometry.Revision != revision)
                throw new InvalidOperationException("Query geometry revision is stale or unavailable.");
        }
        if (!_physics.TryGetBackend(context.World, out var backend))
            throw new InvalidOperationException("The collision scene is unavailable. Missing client geometry is not empty space.");
        return (backend, _transforms.GetWorldPose(context.Frame));
    }

    private static bool Allow(SpatialQueryContext context, SpatialBackendPart part)
        => (context.IgnoredEntity == null || context.IgnoredEntity != part.Entity)
            && (context.Filter?.Allow(context, new SpatialQueryCandidate(part.Entity, part.Surface,
                part.MaterialProfile, part.Revision)) ?? true);
}
