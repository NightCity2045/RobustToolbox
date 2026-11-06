using System;
using System.Collections.Generic;
using System.Diagnostics.CodeAnalysis;
using Robust.Client.GameStates;
using Robust.Shared.GameObjects;
using Robust.Shared.IoC;
using Robust.Shared.Maths;
using Robust.Shared.Network;
using Robust.Shared.Spatial;
using Robust.Shared.Spatial.Networking;
using Robust.Shared.Spatial.Physics;
using Robust.Shared.Timing;

namespace Robust.Client.Spatial;

/// <summary>
/// Captures selected, already replicated entities after complete authoritative
/// state application, before prediction. A selection grants no network visibility.
/// Content owns view authorization, render delay and collision readiness policy.
/// </summary>
public sealed partial class SpatialPoseHistorySystem : EntitySystem
{
    [Dependency] private IClientGameStateManager _states = default!;
    [Dependency] private IClientNetManager _net = default!;
    [Dependency] private SharedTransformSystem _transforms = default!;
    private readonly HashSet<SpatialPoseHistory> _histories = new();

    public override void Initialize()
    {
        base.Initialize();
        _states.GameStateApplied += OnApplied;
        _net.Disconnect += OnDisconnect;
        EntityManager.EntityDeleted += OnDeleted;
        EntityManager.AfterEntityFlush += Clear;
        _transforms.OnGlobalMoveEvent += OnMove;
    }

    public override void Shutdown()
    {
        _states.GameStateApplied -= OnApplied;
        _net.Disconnect -= OnDisconnect;
        EntityManager.EntityDeleted -= OnDeleted;
        EntityManager.AfterEntityFlush -= Clear;
        _transforms.OnGlobalMoveEvent -= OnMove;
        Clear();
        base.Shutdown();
    }

    /// <summary>One disposable, explicitly bounded history per view selection.</summary>
    public SpatialPoseHistory Track(EntityUid world, IEnumerable<EntityUid> selection, SpatialPoseTimelineLimits limits)
    {
        var history = new SpatialPoseHistory(this, world, limits);
        history.SetSelection(selection);
        _histories.Add(history);
        return history;
    }

    internal void Release(SpatialPoseHistory history) => _histories.Remove(history);

    private void Clear()
    {
        foreach (var history in new List<SpatialPoseHistory>(_histories)) history.Dispose();
    }

    private void OnDisconnect(object? sender, NetDisconnectedArgs args) => Clear();
    private void OnDeleted(Entity<MetaDataComponent> entity)
    {
        foreach (var history in new List<SpatialPoseHistory>(_histories))
            if (history.World == entity.Owner) history.Dispose();
            else history.Timeline.Invalidate(entity.Owner);
    }

    private void OnMove(ref MoveEvent args)
    {
        if (!args.ParentChanged) return;
        foreach (var history in _histories)
            if (args.Component.MapUid != history.World) history.Timeline.Invalidate(args.Sender);
    }

    private void OnApplied(GameStateAppliedArgs args)
    {
        foreach (var history in _histories) Capture(history, args.AppliedState.ToSequence);
    }

    private bool TryRead(EntityUid uid, EntityUid world, [NotNullWhen(true)] out TransformComponent? transform,
        [NotNullWhen(true)] out MetaDataComponent? metadata)
    {
        transform = default!; metadata = default!;
        return TryComp(uid, out transform) && TryComp(uid, out metadata)
            && metadata.NetEntity.IsValid() && !metadata.NetEntity.IsClientSide()
            && metadata.EntityLifeStage >= EntityLifeStage.Initialized && metadata.EntityLifeStage < EntityLifeStage.Terminating
            && (metadata.Flags & MetaDataFlags.Detached) == 0 && transform.MapUid == world;
    }

    internal bool IsAvailable(EntityUid uid, EntityUid world) => HasComp<SpatialWorldComponent>(world)
        && TryRead(uid, world, out _, out _);

    private void Capture(SpatialPoseHistory history, GameTick tick)
    {
        if (history.LastAppliedTick is { } last && tick <= last) history.Timeline.Clear();
        history.LastAppliedTick = tick;
        if (!HasComp<SpatialWorldComponent>(history.World)
            || !TryRead(history.World, history.World, out var root, out _))
        {
            history.Timeline.Clear(); history.LastInstallResult = null; return;
        }

        var nodes = new Dictionary<EntityUid, SpatialPoseNode>
        {
            [history.World] = new(history.World, null, root.LocalPose),
        };
        var contained = new Dictionary<EntityUid, bool> { [history.World] = false };
        var unavailable = new HashSet<EntityUid>();
        var path = new List<(EntityUid Uid, TransformComponent Transform, MetaDataComponent Metadata)>();
        var visiting = new HashSet<EntityUid>();
        foreach (var selected in history.Selection)
        {
            path.Clear(); visiting.Clear();
            var cursor = selected;
            var valid = true;
            while (!nodes.ContainsKey(cursor))
            {
                if (unavailable.Contains(cursor) || !visiting.Add(cursor)
                    || !TryRead(cursor, history.World, out var transform, out var metadata)
                    || TryComp<SpatialBodyComponent>(cursor, out var body) && !body.HasDefinition)
                {
                    valid = false; break;
                }
                if (nodes.Count + path.Count >= history.Limits.MaximumEntitiesPerFrame
                    || nodes.Count + path.Count >= history.Limits.MaximumRetainedSamples)
                {
                    history.Timeline.Clear(); history.LastInstallResult = SpatialPoseInstallResult.BudgetExceeded; return;
                }
                path.Add((cursor, transform, metadata)); cursor = transform.ParentUid;
            }
            if (!valid)
            {
                unavailable.Add(cursor);
                foreach (var entry in path) unavailable.Add(entry.Uid);
                continue;
            }
            var inContainer = contained[cursor];
            for (var i = path.Count - 1; i >= 0; i--)
            {
                var (uid, transform, metadata) = path[i];
                inContainer |= (metadata.Flags & MetaDataFlags.InContainer) != 0;
                var mode = !inContainer && TryComp<SpatialBodyComponent>(uid, out var body)
                    && body.Definition.Owner is SpatialMotionOwner.DynamicBody or SpatialMotionOwner.Motor
                    ? SpatialPoseInterpolation.World : SpatialPoseInterpolation.Hierarchy;
                nodes.Add(uid, new SpatialPoseNode(uid, transform.ParentUid, transform.LocalPose, mode));
                contained.Add(uid, inContainer);
            }
        }
        history.LastInstallResult = history.Timeline.Install(tick, nodes.Values);
    }
}

/// <summary>
/// Derived display history, never authoritative or predicted ECS state. Dispose
/// when the view closes. World deletion, entity flush and disconnect dispose it too.
/// A changed selection immediately clears its old samples; parents are dependencies,
/// not additional selectable render objects. All methods require the client thread.
/// </summary>
public sealed class SpatialPoseHistory : IDisposable
{
    private readonly SpatialPoseHistorySystem _owner;
    private readonly int _thread = Environment.CurrentManagedThreadId;
    internal readonly SpatialPoseTimeline Timeline;
    internal HashSet<EntityUid> Selection = new();
    public SpatialPoseTimelineLimits Limits { get; }
    public EntityUid World => Timeline.World;
    public GameTick? LastAppliedTick { get; internal set; }
    public SpatialPoseInstallResult? LastInstallResult { get; internal set; }
    public bool IsDisposed { get; private set; }
    public int RetainedSamples => Timeline.RetainedSamples;

    internal SpatialPoseHistory(SpatialPoseHistorySystem owner, EntityUid world, SpatialPoseTimelineLimits limits)
    {
        _owner = owner; Limits = limits; Timeline = new SpatialPoseTimeline(world, limits);
    }

    public void SetSelection(IEnumerable<EntityUid> selection)
    {
        CheckThread(); ObjectDisposedException.ThrowIf(IsDisposed, this);
        ArgumentNullException.ThrowIfNull(selection);
        var next = new HashSet<EntityUid>();
        var count = 0;
        foreach (var uid in selection)
        {
            if (++count > Limits.MaximumEntitiesPerFrame || count > Limits.MaximumRetainedSamples)
            {
                Selection.Clear(); Timeline.Clear(); LastInstallResult = SpatialPoseInstallResult.BudgetExceeded;
                throw new ArgumentException("Pose selection exceeds its entity budget.", nameof(selection));
            }
            if (!uid.IsValid()) throw new ArgumentException("Invalid pose selection entity.", nameof(selection));
            next.Add(uid);
        }
        if (Selection.SetEquals(next)) return;
        Selection = next; Timeline.Clear(); LastInstallResult = null; LastAppliedTick = null;
    }

    public bool TrySample(EntityUid entity, double renderedTick, out RigidTransform3 pose)
    {
        CheckThread(); pose = default;
        return !IsDisposed && Selection.Contains(entity) && _owner.IsAvailable(entity, World)
            && Timeline.TrySample(entity, renderedTick, out pose);
    }

    public void Dispose()
    {
        CheckThread();
        if (IsDisposed) return;
        IsDisposed = true; Timeline.Clear(); Selection.Clear(); _owner.Release(this);
    }

    private void CheckThread()
    {
        if (_thread != Environment.CurrentManagedThreadId) throw new InvalidOperationException("Pose history requires its owning client thread.");
    }
}
