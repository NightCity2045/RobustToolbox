using System;
using System.Collections.Generic;
using System.Diagnostics.CodeAnalysis;
using Robust.Client.Timing;
using Robust.Shared.GameObjects;
using Robust.Shared.IoC;
using Robust.Shared.Maths;
using Robust.Shared.Network;
using Robust.Shared.Spatial;
using Robust.Shared.Spatial.Networking;

namespace Robust.Client.Spatial;

/// <summary>Published only after the authorized multi-frame replica has been installed atomically.</summary>
[ByRefEvent]
public readonly record struct SpatialGeometryReplicaChanged(ulong Sequence);

/// <summary>Read-only geometry cache; canonical entities and their transforms remain in the normal game state.</summary>
public sealed partial class SpatialGeometryStreamingSystem : EntitySystem
{
    [Dependency] private IClientNetManager _net = default!;
    [Dependency] private IClientGameTiming _timing = default!;
    [Dependency] private SharedTransformSystem _transforms = default!;
    private readonly SpatialGeometryReplica _replica = new();
    private TimeSpan _pendingSince;
    private TimeSpan _lastRequest = TimeSpan.MinValue;

    public ulong Sequence => _replica.Sequence;

    public override void Initialize()
    {
        base.Initialize();
        UpdatesOutsidePrediction = true;
        _net.Disconnect += OnDisconnect;
        SubscribeLocalEvent<SpatialFrameComponent, SpatialGeometryStampChanged>(OnStampChanged);
        _transforms.OnGlobalMoveEvent += OnMove;
        EntityManager.EntityDeleted += OnDeleted;
        EntityManager.AfterEntityFlush += Clear;
    }

    public override void Shutdown()
    {
        _net.Disconnect -= OnDisconnect;
        _transforms.OnGlobalMoveEvent -= OnMove;
        EntityManager.EntityDeleted -= OnDeleted;
        EntityManager.AfterEntityFlush -= Clear;
        Clear();
        base.Shutdown();
    }

    private void Clear()
    {
        _replica.Clear();
        _pendingSince = default;
        _lastRequest = TimeSpan.MinValue;
    }
    private void OnDisconnect(object? sender, NetDisconnectedArgs ev) => Clear();
    private void OnDeleted(Entity<MetaDataComponent> entity) => _replica.InvalidateKnown(entity.Comp.NetEntity, _timing.LastRealTick);
    private void OnStampChanged(EntityUid uid, SpatialFrameComponent component, ref SpatialGeometryStampChanged ev)
        => _replica.Invalidate(GetNetEntity(uid), _timing.LastRealTick);

    private void OnMove(ref MoveEvent ev)
    {
        if (!ev.ParentChanged || TryComp<TransformComponent>(ev.OldPosition.EntityId, out var previous)
            && previous.MapUid == ev.Component.MapUid) return;
        // A world transfer can retain the exact same geometry revision. Cached
        // geometry from the old world must stop certifying collision immediately.
        _replica.InvalidateUnresolved(Resolve, _timing.LastRealTick);
    }

    internal void OnManifest(MsgSpatialGeometryManifest message)
    {
        if (message.MsgChannel != _net.ServerChannel) return;
        if (message.Manifest.Sequence == _replica.Sequence)
        {
            _net.ClientSendMessage(new MsgSpatialGeometryAck { Sequence = _replica.Sequence });
            return;
        }
        if (_replica.Begin(message.Manifest)) _pendingSince = _timing.RealTime;
        if (_replica.NeedsBaseline) RequestBaseline();
    }

    internal void OnFragment(MsgSpatialGeometryFragment message)
    {
        if (message.MsgChannel != _net.ServerChannel) return;
        try
        {
            if (_replica.AddFragment(message.Sequence, message.Frame, message.Offset, message.Data))
            {
                _pendingSince = _timing.RealTime;
                _net.ClientSendMessage(new MsgSpatialGeometryProgress
                    { Sequence = message.Sequence, Frame = message.Frame, Offset = message.Offset });
            }
        }
        catch { RequestBaseline(); throw; }
    }

    public override void Update(float frameTime)
    {
        if (!_net.IsConnected || _timing.ApplyingState) return;
        if (_replica.HasPending && _timing.RealTime - _pendingSince > TimeSpan.FromSeconds(5))
            _replica.RequireBaseline();
        var result = _replica.TryApply(_timing.LastRealTick, Resolve,
            uid => TryComp<SpatialFrameComponent>(uid, out var frame) ? frame.ReceivedGeometryStamp : null);
        if (result == SpatialGeometryApplyResult.NeedBaseline)
        {
            RequestBaseline();
            return;
        }
        if (result != SpatialGeometryApplyResult.Applied) return;
        _net.ClientSendMessage(new MsgSpatialGeometryAck { Sequence = _replica.Sequence });
        var ev = new SpatialGeometryReplicaChanged(_replica.Sequence);
        RaiseLocalEvent(ref ev);
    }

    private EntityUid? Resolve(NetEntity world, NetEntity net)
    {
        if (!TryGetEntity(world, out var map) || !HasComp<SpatialWorldComponent>(map.Value)
            || !TryGetEntity(net, out var uid) || !TryComp<MetaDataComponent>(uid.Value, out var meta)
            || meta.EntityLifeStage < EntityLifeStage.Initialized || meta.EntityLifeStage >= EntityLifeStage.Terminating
            || (meta.Flags & MetaDataFlags.Detached) != 0 || !TryComp<TransformComponent>(uid.Value, out var transform)
            || transform.MapUid != map) return null;
        return uid;
    }

    private void RequestBaseline()
    {
        if (!_net.IsConnected || _lastRequest != TimeSpan.MinValue && _timing.RealTime - _lastRequest < TimeSpan.FromSeconds(1)) return;
        _lastRequest = _timing.RealTime;
        _net.ClientSendMessage(new MsgSpatialGeometryRequestBaseline { LastSequence = _replica.Sequence });
    }

    /// <summary>False means unknown collision data; callers must not substitute an empty scene.</summary>
    public bool IsReady(EntityUid world, Box3 bounds) => _replica.IsCovered(GetNetEntity(world), bounds);

    public bool TryGetSnapshot(EntityUid frame, [NotNullWhen(true)] out SpatialGeometrySnapshot? snapshot)
    {
        if (_replica.TryGetSnapshot(GetNetEntity(frame), out var found) && found != null)
        {
            snapshot = found;
            return true;
        }
        snapshot = null;
        return false;
    }

    public bool IsCurrent(SpatialGeometrySnapshot snapshot)
        => TryGetSnapshot(snapshot.OwnerFrame, out var current) && ReferenceEquals(snapshot, current);

    public IReadOnlyList<SpatialGeometrySnapshot> CaptureSnapshots(EntityUid world)
    {
        var result = new List<SpatialGeometrySnapshot>();
        var netWorld = GetNetEntity(world);
        foreach (var frame in _replica.KnownFrames)
            if (Resolve(netWorld, frame) is { } uid && TryGetSnapshot(uid, out var snapshot)) result.Add(snapshot);
        return result.AsReadOnly();
    }
}
