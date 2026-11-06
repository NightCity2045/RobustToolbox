using System;
using System.Collections.Generic;
using System.Numerics;
using Robust.Shared.GameObjects;
using Robust.Shared.Maths;
using Robust.Shared.Timing;

namespace Robust.Shared.Spatial.Networking;

/// <summary>
/// Hierarchy members follow their interpolated parent, including rotational arcs.
/// Independent physical bodies interpolate their world poses, avoiding a second
/// application of carrier motion. This is presentation policy, never a body owner.
/// </summary>
public enum SpatialPoseInterpolation : byte { Hierarchy, World }
public readonly record struct SpatialPoseNode(EntityUid Entity, EntityUid? Parent, RigidTransform3 LocalPose,
    SpatialPoseInterpolation Interpolation = SpatialPoseInterpolation.Hierarchy);
public readonly record struct SpatialPoseTimelineLimits(int MaximumFrames, int MaximumEntitiesPerFrame, int MaximumRetainedSamples);
public enum SpatialPoseInstallResult : byte { Installed, Older, BudgetExceeded }

/// <summary>
/// Bounded derived presentation history of coherent authoritative transform batches
/// for one existing world. It never writes ECS or steps physics. Submit after a
/// complete state application, with authorized parents included. Clear on world
/// unload/reconnect/tick reset. Content chooses render delay and observes geometry
/// readiness separately; this class does not make missing collision data safe.
/// </summary>
public sealed class SpatialPoseTimeline
{
    private readonly SpatialPoseTimelineLimits _limits;
    private readonly List<Frame> _frames = new();
    private readonly int _thread = Environment.CurrentManagedThreadId;
    public EntityUid World { get; }
    public int RetainedSamples { get; private set; }
    public int FrameCount => _frames.Count;

    public SpatialPoseTimeline(EntityUid world, SpatialPoseTimelineLimits limits)
    {
        if (!world.IsValid() || limits.MaximumFrames < 2 || limits.MaximumFrames > 1024
            || limits.MaximumEntitiesPerFrame <= 0 || limits.MaximumRetainedSamples <= 0)
            throw new ArgumentOutOfRangeException(nameof(limits));
        World = world; _limits = limits;
    }

    public SpatialPoseInstallResult Install(GameTick tick, IEnumerable<SpatialPoseNode> nodes)
    {
        CheckThread(); ArgumentNullException.ThrowIfNull(nodes);
        if (_frames.Count != 0 && tick <= _frames[^1].Tick) return SpatialPoseInstallResult.Older;
        var copy = new Dictionary<EntityUid, SpatialPoseNode>();
        foreach (var node in nodes)
        {
            if (copy.Count >= _limits.MaximumEntitiesPerFrame || copy.Count >= _limits.MaximumRetainedSamples)
            {
                Clear();
                return SpatialPoseInstallResult.BudgetExceeded;
            }
            if (!node.Entity.IsValid() || !node.LocalPose.IsValid || !Enum.IsDefined(node.Interpolation)
                || node.Parent is { } parent && !parent.IsValid() || !copy.TryAdd(node.Entity, node))
                throw new ArgumentException("Invalid or duplicated pose sample.");
        }
        if (!copy.TryGetValue(World, out var root) || root.Parent != null)
            throw new ArgumentException("A pose batch must contain its world root.");
        var poses = ResolveWorldPoses(copy);
        while (_frames.Count != 0 && (_frames.Count >= _limits.MaximumFrames
               || RetainedSamples > _limits.MaximumRetainedSamples - copy.Count))
        {
            RetainedSamples -= _frames[0].Nodes.Count; _frames.RemoveAt(0);
        }
        _frames.Add(new Frame(tick, copy, poses)); RetainedSamples += copy.Count;
        return SpatialPoseInstallResult.Installed;
    }

    /// <summary>
    /// Samples only entities authorized in the latest batch. No extrapolation:
    /// times after the last sample hold that sample; times before retained history
    /// fail. A new entity is not invented in earlier samples. Hierarchy/ownership
    /// discontinuities snap to the newer coherent pose rather than mixing parents.
    /// </summary>
    public bool TrySample(EntityUid entity, double renderedTick, out RigidTransform3 pose)
    {
        CheckThread(); pose = default;
        if (!double.IsFinite(renderedTick) || _frames.Count == 0 || !_frames[^1].Nodes.ContainsKey(entity)
            || renderedTick < _frames[0].Tick.Value) return false;
        if (renderedTick >= _frames[^1].Tick.Value) return _frames[^1].Poses.TryGetValue(entity, out pose);
        var upper = 1;
        while (_frames[upper].Tick.Value < renderedTick) upper++;
        var a = _frames[upper - 1]; var b = _frames[upper];
        if (renderedTick == a.Tick.Value) return a.Poses.TryGetValue(entity, out pose);
        if (renderedTick == b.Tick.Value) return b.Poses.TryGetValue(entity, out pose);
        if (!a.Nodes.ContainsKey(entity) || !b.Nodes.ContainsKey(entity)) return false;
        var fraction = (float) ((renderedTick - a.Tick.Value) / (b.Tick.Value - a.Tick.Value));
        var chain = new List<EntityUid>();
        var cursor = entity;
        var parentPose = RigidTransform3.Identity;
        while (true)
        {
            var next = b.Nodes[cursor];
            if (!a.Nodes.TryGetValue(cursor, out var previous) || next.Interpolation != previous.Interpolation
                || next.Interpolation == SpatialPoseInterpolation.Hierarchy && next.Parent != previous.Parent)
            {
                pose = b.Poses[entity]; return true;
            }
            if (next.Interpolation == SpatialPoseInterpolation.World)
            {
                parentPose = Interpolate(a.Poses[cursor], b.Poses[cursor], fraction); break;
            }
            chain.Add(cursor);
            if (next.Parent is not { } parent) break;
            cursor = parent;
        }
        for (var i = chain.Count - 1; i >= 0; i--)
        {
            var uid = chain[i];
            parentPose = parentPose.Compose(Interpolate(a.Nodes[uid].LocalPose, b.Nodes[uid].LocalPose, fraction));
        }
        pose = parentPose; return true;
    }

    /// <summary>
    /// Immediate local authorization/lifecycle removal, including dependent children.
    /// No retained frame can subsequently produce the revoked presentation. A new
    /// coherent batch may explicitly include the entity again after reauthorization.
    /// </summary>
    public void Invalidate(EntityUid entity)
    {
        CheckThread();
        foreach (var frame in _frames)
        {
            var removed = new HashSet<EntityUid> { entity };
            // Resolve dependency closure without relying on input order.
            var children = new Dictionary<EntityUid, List<EntityUid>>();
            foreach (var node in frame.Nodes.Values)
                if (node.Parent is { } parent)
                {
                    if (!children.TryGetValue(parent, out var list)) children.Add(parent, list = new List<EntityUid>());
                    list.Add(node.Entity);
                }
            var queue = new Queue<EntityUid>(); queue.Enqueue(entity);
            while (queue.TryDequeue(out var uid))
                if (children.TryGetValue(uid, out var list))
                    foreach (var child in list) if (removed.Add(child)) queue.Enqueue(child);
            foreach (var uid in removed)
                if (frame.Nodes.Remove(uid)) { frame.Poses.Remove(uid); RetainedSamples--; }
        }
    }

    public void Clear() { CheckThread(); _frames.Clear(); RetainedSamples = 0; }

    private Dictionary<EntityUid, RigidTransform3> ResolveWorldPoses(Dictionary<EntityUid, SpatialPoseNode> nodes)
    {
        var poses = new Dictionary<EntityUid, RigidTransform3>();
        foreach (var uid in nodes.Keys)
        {
            if (poses.ContainsKey(uid)) continue;
            var path = new List<EntityUid>(); var seen = new HashSet<EntityUid>();
            var cursor = uid; var parentPose = RigidTransform3.Identity;
            while (!poses.TryGetValue(cursor, out parentPose))
            {
                if (!nodes.TryGetValue(cursor, out var node) || !seen.Add(cursor))
                    throw new ArgumentException("Pose batch has a missing parent or hierarchy cycle.");
                path.Add(cursor);
                if (node.Parent is not { } parent)
                {
                    if (cursor != World) throw new ArgumentException("Every pose must belong to the selected world.");
                    parentPose = RigidTransform3.Identity; break;
                }
                cursor = parent;
            }
            for (var i = path.Count - 1; i >= 0; i--)
            {
                cursor = path[i]; parentPose = parentPose.Compose(nodes[cursor].LocalPose); poses.Add(cursor, parentPose);
            }
        }
        return poses;
    }

    private static RigidTransform3 Interpolate(RigidTransform3 a, RigidTransform3 b, float fraction)
    {
        // Double intermediates avoid overflowing (b - a) for distant finite poses.
        static float Blend(float a, float b, float t) => (float) ((1d - t) * a + (double) t * b);
        var p = new Vector3(Blend(a.Position.X, b.Position.X, fraction), Blend(a.Position.Y, b.Position.Y, fraction),
            Blend(a.Position.Z, b.Position.Z, fraction));
        return new RigidTransform3(p, Quaternion.Slerp(a.Rotation, b.Rotation, fraction));
    }

    private void CheckThread()
    {
        if (_thread != Environment.CurrentManagedThreadId) throw new InvalidOperationException("Pose history requires its owning client thread.");
    }
    private sealed record Frame(GameTick Tick, Dictionary<EntityUid, SpatialPoseNode> Nodes, Dictionary<EntityUid, RigidTransform3> Poses);
}
