using System;
using System.Collections.Generic;
using System.Linq;
using Robust.Shared.GameObjects;

namespace Robust.Shared.Spatial;

/// <summary>Owns validated canonical frame geometry and detached revisioned captures.</summary>
public sealed class SharedGeometrySystem : EntitySystem
{
    public override void Initialize()
    {
        base.Initialize();
        SubscribeLocalEvent<SpatialGeometryComponent, ComponentInit>(OnInit);
    }

    public void ConfigureInitialGeometry(Entity<SpatialGeometryComponent> frame,
        IEnumerable<BoundarySegment> boundaries, IEnumerable<SlabSurface> slabs, ulong revision = 0)
    {
        if (MetaData(frame).EntityLifeStage != EntityLifeStage.PreInit)
            throw new InvalidOperationException("Initial geometry must be configured before entity initialization.");
        if (frame.Comp._schema != SpatialGeometryComponent.CurrentSchema)
            throw new InvalidOperationException($"Unsupported spatial geometry schema {frame.Comp._schema}.");
        var snapshot = new SpatialGeometrySnapshot(frame, revision,
            Comp<SpatialFrameComponent>(frame)._layers, boundaries, slabs);
        // Validate everything before replacing any canonical collection.
        frame.Comp._boundaries = snapshot.Boundaries.Values.ToList();
        frame.Comp._slabs = snapshot.Slabs.Values.ToList();
        frame.Comp._revision = revision;
    }

    /// <summary>Capture once for a geometry job or consumer update, not for each individual raycast.</summary>
    public SpatialGeometrySnapshot CaptureSnapshot(EntityUid frame)
    {
        if (TerminatingOrDeleted(frame))
            throw new InvalidOperationException("Cannot capture geometry of a terminating frame.");
        var geometry = Comp<SpatialGeometryComponent>(frame);
        if (geometry._schema != SpatialGeometryComponent.CurrentSchema)
            throw new InvalidOperationException($"Unsupported spatial geometry schema {geometry._schema}.");
        return new SpatialGeometrySnapshot(frame, geometry._revision, Comp<SpatialFrameComponent>(frame)._layers,
            geometry._boundaries, geometry._slabs);
    }

    /// <summary>Resolve a persistent semantic surface independently of any mesh or physics shape index.</summary>
    public IReadOnlyList<SpatialSolidPart> GetSurfaceParts(SpatialGeometrySnapshot snapshot, SpatialSurfaceReference surface)
    {
        if (snapshot.OwnerFrame != surface.OwnerFrame)
            throw new InvalidOperationException("Surface reference belongs to a different frame.");
        return snapshot.GetSurfaceParts(surface.Surface);
    }

    private void OnInit(EntityUid uid, SpatialGeometryComponent component, ComponentInit args)
        => CaptureSnapshot(uid);
}
