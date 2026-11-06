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
    private readonly SpatialSnapshotQueries _queries;
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
        World = world;
        Tick = tick;
        Bodies = bodies;
        Poses = poses;
        Geometries = geometries;
        ShapeCount = shapeCount;
        _queries = new SpatialSnapshotQueries(world, poses, geometries, () => IsAvailable,
            () => owner.BeginQuery(this), owner.EndQuery);
    }

    public bool TryGetPose(EntityUid entity, out RigidTransform3 pose)
        => Poses.TryGetValue(entity, out pose);

    public bool TryGetGeometry(EntityUid frame, [NotNullWhen(true)] out SpatialGeometrySnapshot? snapshot)
        => Geometries.TryGetValue(frame, out snapshot);

    public bool TryGetBodyState(EntityUid entity, out SpatialHistoricalBodyState state)
    {
        foreach (var body in Bodies)
        {
            if (body.Entity == entity)
            {
                state = new SpatialHistoricalBodyState(body.State.Pose, body.State.LinearVelocity, body.State.AngularVelocity,
                    body.DefinitionRevision);
                return true;
            }
            foreach (var member in body.AnchoredMembers)
            {
                if (member.Key != entity || !Poses.TryGetValue(entity, out var pose)) continue;
                state = new SpatialHistoricalBodyState(pose, body.State.LinearVelocity
                    + Vector3.Cross(body.State.AngularVelocity, pose.Position - body.State.Pose.Position), body.State.AngularVelocity, member.Value);
                return true;
            }
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

    public SpatialHit? RayCast(SpatialQueryContext context, Vector3 origin, Vector3 direction, float distance)
        => _queries.RayCast(context, origin, direction, distance);
    public SpatialHit? FindSurface(SpatialQueryContext context, Vector3 origin, Vector3 direction, float distance)
        => _queries.FindSurface(context, origin, direction, distance);
    public IReadOnlyList<SpatialHit> ShapeCast(SpatialQueryContext context, SpatialShape shape, RigidTransform3 pose,
        Vector3 translation, Vector3 angularDisplacement) => _queries.ShapeCast(context, shape, pose, translation, angularDisplacement);
    public IReadOnlyList<SpatialHit> Overlap(SpatialQueryContext context, SpatialShape shape, RigidTransform3 pose)
        => _queries.Overlap(context, shape, pose);
    public bool IsVolumeFree(SpatialQueryContext context, SpatialShape shape, RigidTransform3 pose)
        => _queries.IsVolumeFree(context, shape, pose);
    public IReadOnlyList<SpatialOverlap> OverlapContacts(SpatialQueryContext context, SpatialShape shape, RigidTransform3 pose)
        => _queries.OverlapContacts(context, shape, pose);
}
