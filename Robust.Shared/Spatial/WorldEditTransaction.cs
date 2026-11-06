using System;
using System.Collections.Generic;
using System.Linq;
using Robust.Shared.GameObjects;
using Robust.Shared.Maths;
using Robust.Shared.Spatial.Physics;

namespace Robust.Shared.Spatial;

/// <summary>A detached full replacement for one frame, guarded by its captured revision.</summary>
public sealed class SpatialGeometryEdit
{
    internal SpatialGeometrySnapshot Draft { get; }
    internal object ExpectedIdentity { get; }
    public EntityUid Frame => Draft.OwnerFrame;
    public ulong ExpectedRevision { get; }

    public SpatialGeometryEdit(SpatialGeometrySnapshot original,
        IEnumerable<BoundarySegment>? boundaries = null, IEnumerable<SlabSurface>? slabs = null,
        IEnumerable<SpatialTileChunk>? tileChunks = null, IReadOnlyDictionary<LayerId, LayerDefinition>? layers = null)
        : this(original.OwnerFrame, original.Revision, original.CaptureIdentity ?? throw new ArgumentException("An edit requires a captured live geometry snapshot."), layers ?? original.Layers,
            boundaries ?? original.Boundaries.Values, slabs ?? original.Slabs.Values, tileChunks ?? original.TileChunks.Values) { }

    private SpatialGeometryEdit(EntityUid frame, ulong expectedRevision, object expectedIdentity, IReadOnlyDictionary<LayerId, LayerDefinition> layers,
        IEnumerable<BoundarySegment> boundaries, IEnumerable<SlabSurface> slabs, IEnumerable<SpatialTileChunk> tileChunks)
    {
        ExpectedRevision = expectedRevision;
        ExpectedIdentity = expectedIdentity;
        Draft = new SpatialGeometrySnapshot(frame, checked(expectedRevision + 1), layers, boundaries, slabs, tileChunks, expectedIdentity);
    }
}

public enum WorldEditStatus : byte { Prepared, Queued, Committed, Rejected, Cancelled }

/// <summary>Local solid bounds requiring invalidation; surface IDs also cover empty/open surfaces.</summary>
public readonly record struct SpatialGeometryRegion(LayerId Layer, Box3 Bounds);

public sealed class SpatialGeometryChange
{
    public SpatialGeometrySnapshot Before { get; }
    public SpatialGeometrySnapshot After { get; }
    public IReadOnlyList<SurfaceId> Surfaces { get; }
    public IReadOnlyList<SpatialChunkKey> TileChunks { get; }
    public IReadOnlyList<LayerId> Layers { get; }
    public IReadOnlyList<SpatialGeometryRegion> Regions { get; }

    internal SpatialGeometryChange(SpatialGeometrySnapshot before, SpatialGeometrySnapshot after,
        IEnumerable<SurfaceId> surfaces, IEnumerable<SpatialChunkKey> chunks, IEnumerable<LayerId> layers)
    {
        Before = before;
        After = after;
        var changed = surfaces.ToHashSet();
        Surfaces = Array.AsReadOnly(changed.ToArray());
        TileChunks = Array.AsReadOnly(chunks.ToArray());
        Layers = Array.AsReadOnly(layers.ToArray());
        var regions = new HashSet<SpatialGeometryRegion>();
        foreach (var snapshot in new[] { before, after })
            foreach (var part in snapshot.SolidParts)
                if (changed.Contains(part.Surface.Surface))
                {
                    var layer = part.Boundary is { } boundary ? boundary.LayerId : snapshot.Slabs[part.Surface.Surface].Layer;
                    regions.Add(new SpatialGeometryRegion(layer, part.Bounds));
                }
        Regions = Array.AsReadOnly(regions.ToArray());
    }
}

/// <summary>
/// Owns validated geometry and prepared collision resources until one boundary
/// commit or cancellation. Dispose abandoned work. No gameplay resources are owned.
/// </summary>
public sealed class WorldEditTransaction : IDisposable
{
    internal readonly SharedGeometrySystem Service;
    internal readonly IReadOnlyList<SharedSpatialPhysicsSystem.PreparedGeometryBody> Physics;
    internal readonly IReadOnlyList<SpatialGeometryComponent> Components;
    internal readonly IReadOnlyList<SpatialFrameComponent> Frames;
    internal readonly SpatialPhysicsWorldComponent? PhysicsProfile;
    internal bool Notified;
    public Guid Id { get; } = Guid.NewGuid();
    public EntityUid World { get; }
    public WorldEditStatus Status { get; internal set; }
    public string? Failure { get; internal set; }
    public IReadOnlyList<SpatialGeometryChange> Changes { get; }

    internal WorldEditTransaction(SharedGeometrySystem service, EntityUid world, SpatialGeometryChange[] changes,
        SpatialGeometryComponent[] components, SpatialFrameComponent[] frames,
        SharedSpatialPhysicsSystem.PreparedGeometryBody[] physics, SpatialPhysicsWorldComponent? physicsProfile)
    {
        Service = service;
        World = world;
        Changes = Array.AsReadOnly(changes);
        Components = Array.AsReadOnly(components);
        Frames = Array.AsReadOnly(frames);
        Physics = Array.AsReadOnly(physics);
        PhysicsProfile = physicsProfile;
    }

    public IReadOnlyList<SpatialGeometryEdit> CreateInverse()
    {
        if (Status != WorldEditStatus.Committed)
            throw new InvalidOperationException("Only a committed structural change has an inverse.");
        return Array.AsReadOnly(Changes.Select(c => new SpatialGeometryEdit(c.After,
            c.Before.Boundaries.Values, c.Before.Slabs.Values, c.Before.TileChunks.Values, c.Before.Layers)).ToArray());
    }

    public void Dispose() => Service.CancelTransaction(this);
}

/// <summary>Queued work reached a terminal state. Committed revisions are already queryable.</summary>
[ByRefEvent]
public readonly record struct WorldEditCompleted(WorldEditTransaction Transaction);
