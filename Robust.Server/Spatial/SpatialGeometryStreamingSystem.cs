using System;
using System.Collections.Generic;
using System.Linq;
using System.Security.Cryptography;
using Robust.Shared;
using Robust.Shared.Configuration;
using Robust.Shared.GameObjects;
using Robust.Shared.IoC;
using Robust.Shared.Maths;
using Robust.Shared.Network;
using Robust.Shared.Player;
using Robust.Shared.Spatial;
using Robust.Shared.Spatial.Networking;
using Robust.Shared.Timing;

namespace Robust.Server.Spatial;

internal sealed class SpatialGeometrySelection
{
    public readonly HashSet<SpatialSurfaceReference> Surfaces = new();
    public readonly List<(EntityUid World, Box3 Bounds)> Views = new();
    public readonly HashSet<EntityUid> Viewers = new();
    public bool Complete = true;
    public void Clear() { Surfaces.Clear(); Views.Clear(); Viewers.Clear(); Complete = true; }
}

/// <summary>Bounded, acknowledged geometry transfer after the normal PVS state snapshot.</summary>
public sealed partial class SpatialGeometryStreamingSystem : EntitySystem
{
    [Dependency] private IServerNetManager _net = default!;
    [Dependency] private IGameTiming _timing = default!;
    [Dependency] private IConfigurationManager _config = default!;
    [Dependency] private SharedGeometrySystem _geometry = default!;
    private readonly Dictionary<INetChannel, SessionTransfer> _sessions = new();
    private const int MaximumOutstandingFragments = 16;

    private sealed record Payload(EntityUid Frame, EntityUid World, HashSet<SurfaceId> Surfaces,
        EntityUid[] Sources, SpatialGeometryFrameManifest Description, byte[] Bytes);
    private sealed class Transfer(SpatialGeometryManifest manifest, List<Payload> payloads, HashSet<EntityUid> viewers)
    {
        public readonly SpatialGeometryManifest Manifest = manifest;
        public readonly List<Payload> Payloads = payloads;
        public readonly HashSet<EntityUid> Viewers = viewers;
        public readonly HashSet<(NetEntity Frame, int Part, int Offset)> Outstanding = new();
        public int Frame;
        public int Offset;
        public TimeSpan Progress;
        public bool AllSent => Frame == Payloads.Count;
    }
    private sealed class SessionTransfer
    {
        public ulong NextSequence;
        public ulong Acknowledged;
        public bool ForceBaseline;
        public Transfer? Pending;
        public List<Payload> Active = new();
        public IReadOnlyList<SpatialGeometryCoverage> Coverage = Array.Empty<SpatialGeometryCoverage>();
        public TimeSpan LastRequest = TimeSpan.MinValue;
    }

    public override void Initialize()
    {
        base.Initialize();
        _net.Disconnect += OnDisconnect;
        EntityManager.AfterEntityFlush += Clear;
    }

    public override void Shutdown()
    {
        _net.Disconnect -= OnDisconnect;
        EntityManager.AfterEntityFlush -= Clear;
        Clear();
        base.Shutdown();
    }
    private void Clear() => _sessions.Clear();
    private void OnDisconnect(object? sender, NetDisconnectedArgs ev) => _sessions.Remove(ev.Channel);

    internal void Publish(ICommonSession session, SpatialGeometrySelection selection, Func<EntityUid, bool> allowed)
    {
        var channel = session.Channel;
        if (channel is DummyChannel || !channel.IsConnected) return;
        if (!_sessions.TryGetValue(channel, out var state))
        {
            if (selection.Views.Count == 0) return;
            _sessions.Add(channel, state = new SessionTransfer());
        }
        if (state.Pending is { } pending)
        {
            // Range churn alone can finish an already-authorized bounded transfer.
            // Removed viewers, revoked source access and changed frame generations
            // stop further fragments immediately and replace the pending selection.
            if (!pending.Viewers.IsSubsetOf(selection.Viewers) || pending.Payloads.Any(p => !Current(p, allowed))
                || _timing.RealTime - pending.Progress > TimeSpan.FromSeconds(10))
            {
                state.Pending = null;
                state.ForceBaseline = true;
            }
            else
            {
                Pump(channel, state);
                return;
            }
        }

        var payloads = BuildPayloads(selection, state.Active, allowed);
        var coverage = selection.Complete
            ? selection.Views.Select(v => new SpatialGeometryCoverage(GetNetEntity(v.World), v.Bounds)).ToArray()
            : Array.Empty<SpatialGeometryCoverage>();
        if (!state.ForceBaseline && state.Acknowledged != 0 && coverage.SequenceEqual(state.Coverage)
            && payloads.Select(p => p.Description).SequenceEqual(state.Active.Select(p => p.Description))) return;
        var manifest = new SpatialGeometryManifest(checked(++state.NextSequence), state.ForceBaseline ? 0 : state.Acknowledged,
            _timing.CurTick, payloads.Select(p => p.Description), coverage);
        state.Pending = new Transfer(manifest, payloads, new HashSet<EntityUid>(selection.Viewers)) { Progress = _timing.RealTime };
        _net.ServerSendMessage(new MsgSpatialGeometryManifest { Manifest = manifest }, channel);
        Pump(channel, state, new MsgSpatialGeometryManifest { Manifest = manifest }.EstimateBufferSize());
    }

    private bool Current(Payload payload, Func<EntityUid, bool> allowed)
        => !TerminatingOrDeleted(payload.Frame) && TryComp<SpatialGeometryComponent>(payload.Frame, out var geometry)
            && geometry.Running && geometry.WireGeneration == payload.Description.Stamp.Generation
            && geometry.Revision == payload.Description.Stamp.Revision
            && Transform(payload.Frame).MapUid == payload.World && payload.Sources.All(allowed);

    private List<Payload> BuildPayloads(SpatialGeometrySelection selection, List<Payload> previous, Func<EntityUid, bool> allowed)
    {
        var payloads = new List<Payload>();
        foreach (var group in selection.Surfaces.GroupBy(s => s.OwnerFrame).OrderBy(g => g.Key))
        {
            var frame = group.Key;
            if (!allowed(frame) || !TryComp<SpatialGeometryComponent>(frame, out var geometry) || !geometry.Running
                || Transform(frame).MapUid is not { } world) continue;
            var surfaces = group.Select(s => s.Surface).ToHashSet();
            var stamp = new SpatialGeometryStamp(geometry.WireGeneration, geometry.Revision);
            var old = previous.Where(p => p.Frame == frame && p.World == world && p.Description.Stamp == stamp
                && p.Surfaces.SetEquals(surfaces)).ToArray();
            if (old.Length != 0) { payloads.AddRange(old); continue; }
            var snapshot = SpatialGeometryWireCodec.Select(_geometry.CaptureSnapshot(frame), surfaces);
            var sources = snapshot.Boundaries.Values.Select(b => b.Entity).Concat(snapshot.Slabs.Values.Select(s => s.Entity))
                .Where(e => e != null).Select(e => e!.Value).Append(frame).Distinct().ToArray();
            if (!sources.All(allowed)) { selection.Complete = false; continue; }
            var parts = SpatialGeometryWireCodec.EncodeParts(snapshot, uid => GetNetEntity(uid));
            for (var part = 0; part < parts.Count; part++)
            {
                var bytes = parts[part];
                var description = new SpatialGeometryFrameManifest(GetNetEntity(world), GetNetEntity(frame), stamp,
                    Convert.ToHexString(SHA256.HashData(bytes)), bytes.Length, part, parts.Count);
                payloads.Add(new Payload(frame, world, surfaces, sources, description, bytes));
            }
        }
        return payloads;
    }

    private void Pump(INetChannel channel, SessionTransfer state, int alreadyQueued = 0)
    {
        if (state.Pending is not { } transfer) return;
        var budget = Math.Clamp(_config.GetCVar(CVars.NetSpatialGeometryBytes), 65536, 4 * 1024 * 1024) - alreadyQueued;
        while (!transfer.AllSent && transfer.Outstanding.Count < MaximumOutstandingFragments)
        {
            var payload = transfer.Payloads[transfer.Frame];
            if (!state.ForceBaseline && state.Active.Any(p => p.Description == payload.Description))
            {
                transfer.Frame++;
                transfer.Offset = 0;
                continue;
            }
            var length = Math.Min(SpatialGeometryReplica.FragmentSize, payload.Bytes.Length - transfer.Offset);
            if (budget < length + 24) break;
            _net.ServerSendMessage(new MsgSpatialGeometryFragment
            {
                Sequence = transfer.Manifest.Sequence, Frame = payload.Description.Frame, Part = payload.Description.Part, Offset = transfer.Offset,
                Data = payload.Bytes.AsSpan(transfer.Offset, length).ToArray()
            }, channel);
            transfer.Outstanding.Add((payload.Description.Frame, payload.Description.Part, transfer.Offset));
            budget -= length + 24;
            transfer.Offset += length;
            if (transfer.Offset == payload.Bytes.Length) { transfer.Frame++; transfer.Offset = 0; }
        }
    }

    internal void OnProgress(MsgSpatialGeometryProgress message)
    {
        if (_sessions.TryGetValue(message.MsgChannel, out var state) && state.Pending is { } transfer
            && transfer.Manifest.Sequence == message.Sequence && transfer.Outstanding.Remove((message.Frame, message.Part, message.Offset)))
            transfer.Progress = _timing.RealTime;
    }

    internal void OnAck(MsgSpatialGeometryAck message)
    {
        if (!_sessions.TryGetValue(message.MsgChannel, out var state) || state.Pending is not { } transfer
            || transfer.Manifest.Sequence != message.Sequence || !transfer.AllSent || transfer.Outstanding.Count != 0) return;
        state.Acknowledged = message.Sequence;
        state.Active = transfer.Payloads;
        state.Coverage = transfer.Manifest.Coverage;
        state.Pending = null;
        state.ForceBaseline = false;
    }

    internal void OnBaselineRequest(MsgSpatialGeometryRequestBaseline message)
    {
        if (!_sessions.TryGetValue(message.MsgChannel, out var state)
            || state.LastRequest != TimeSpan.MinValue && _timing.RealTime - state.LastRequest < TimeSpan.FromSeconds(1)) return;
        state.LastRequest = _timing.RealTime;
        state.Pending = null;
        state.ForceBaseline = true;
    }
}
