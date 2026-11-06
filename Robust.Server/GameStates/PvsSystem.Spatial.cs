using System;
using System.Collections.Generic;
using System.Numerics;
using Robust.Shared;
using Robust.Shared.GameObjects;
using Robust.Shared.Maths;
using Robust.Shared.Player;
using Robust.Shared.Spatial;
using Robust.Shared.Spatial.Physics;

namespace Robust.Server.GameStates;

/// <summary>
/// Read-only server policy hook for normal spatial interest. Called during parallel
/// PVS generation. The visibility mask is enforced before this hook. Container
/// descendants default to denied; content may authorize a particular session.
/// This does not grant view subscriptions or change explicit content PVS overrides.
/// </summary>
[ByRefEvent]
public struct SpatialInterestVisibilityEvent(ICommonSession session, EntityUid entity, bool allowed)
{
    public readonly ICommonSession Session = session;
    public readonly EntityUid Entity = entity;
    public bool Allowed = allowed;
}

internal sealed partial class PvsSystem
{
    private void AddSpatialEntities(PvsSession session)
    {
        var selected = new Dictionary<EntityUid, double>();
        var allowedCache = new Dictionary<EntityUid, bool>();
        var candidates = new HashSet<EntityUid>();
        var surfaces = new HashSet<SpatialSurfaceReference>();
        var seconds = FiniteNonnegative(_configManager.GetCVar(CVars.NetSpatialPreloadSeconds));
        var maximum = FiniteNonnegative(_configManager.GetCVar(CVars.NetSpatialPreloadDistance));

        // These viewers come exclusively from the attached entity and authorized
        // server ViewSubscriptions. No client-provided camera/world is accepted here.
        foreach (var viewer in session.Viewers)
        {
            if (viewer.Comp1.MapUid is not { } map || !HasComp<SpatialWorldComponent>(map)) continue;
            var position = _transform.GetWorldPose(viewer.Comp1).Position;
            var offset = viewer.Comp2?.Offset ?? Vector2.Zero;
            position += new Vector3(offset, 0);
            var range = Math.Max(_viewSize, _priorityViewSize) * 0.5f * Math.Max(viewer.Comp2?.PvsScale ?? 1, 0.1f);
            if (!float.IsFinite(range) || range <= 0) continue;
            var preload = GetSpatialPreload(viewer.Owner, position, seconds, maximum);
            var target = position + preload;
            var extent = new Vector3(range);
            var bounds = new Box3(Vector3.Min(position, target) - extent, Vector3.Max(position, target) + extent);
            candidates.Clear();
            surfaces.Clear();
            _spatialInterest.CollectPrepared(map, bounds, Allowed, candidates, surfaces);
            foreach (var uid in candidates)
            {
                var relative = _transform.GetWorldPose(uid).Position - position;
                var distance = (double) relative.X * relative.X + (double) relative.Y * relative.Y + (double) relative.Z * relative.Z;
                if (!selected.TryGetValue(uid, out var old) || distance < old) selected[uid] = distance;
            }
        }

        var sorted = new List<EntityUid>(selected.Keys);
        sorted.Sort((a, b) =>
        {
            var distance = selected[a].CompareTo(selected[b]);
            return distance != 0 ? distance : a.CompareTo(b);
        });
        var visiting = new HashSet<EntityUid>();
        var added = new HashSet<EntityUid>();
        foreach (var uid in sorted) AddWithDependencies(uid);
        return;

        bool Allowed(EntityUid uid)
        {
            if (allowedCache.TryGetValue(uid, out var cached)) return cached;
            if (!_metaQuery.TryGetComponent(uid, out var meta) || meta.EntityLifeStage >= EntityLifeStage.Terminating
                || (session.VisMask & meta.VisibilityMask) != meta.VisibilityMask
                || !_xformQuery.TryGetComponent(uid, out var transform))
                return allowedCache[uid] = false;
            var allowed = (meta.Flags & MetaDataFlags.InContainer) == 0;
            var ev = new SpatialInterestVisibilityEvent(session.Session, uid, allowed);
            RaiseLocalEvent(ref ev);
            return allowedCache[uid] = ev.Allowed
                && (!transform.ParentUid.IsValid() || Allowed(transform.ParentUid));
        }

        bool AddWithDependencies(EntityUid uid)
        {
            if (added.Contains(uid)) return true;
            if (!Allowed(uid) || !visiting.Add(uid)) return false;
            try
            {
                var transform = _xformQuery.GetComponent(uid);
                if (transform.ParentUid.IsValid() && !AddWithDependencies(transform.ParentUid)) return false;
                if (TryComp<SpatialMotorComponent>(uid, out var motor) && motor.Support is { } support)
                {
                    if (support.Source.Surface is { } surface && surface.OwnerFrame != uid
                        && !AddWithDependencies(surface.OwnerFrame)) return false;
                    if (support.Source.Entity is { } entity && entity != uid && !AddWithDependencies(entity)) return false;
                }
                if (!AddEntity(session, (uid, _metaQuery.GetComponent(uid)), session.FromTick)) return false;
                added.Add(uid);
                return true;
            }
            finally { visiting.Remove(uid); }
        }
    }

    private Vector3 GetSpatialPreload(EntityUid viewer, Vector3 position, float seconds, float maximum)
    {
        // An independent body already stores its complete world velocity. For a
        // camera parented to a vehicle, use the first body's point velocity once.
        while (_xformQuery.TryGetComponent(viewer, out var transform))
        {
            if (TryComp<SpatialBodyComponent>(viewer, out var body))
            {
                var velocity = body.LinearVelocity + Vector3.Cross(body.AngularVelocity,
                    position - _transform.GetWorldPose(transform).Position);
                var length = Math.Sqrt((double) velocity.X * velocity.X + (double) velocity.Y * velocity.Y + (double) velocity.Z * velocity.Z);
                if (!double.IsFinite(length) || length == 0) return Vector3.Zero;
                var scale = Math.Min(seconds, maximum / length);
                return velocity * (float) scale;
            }
            if (!transform.ParentUid.IsValid()) break;
            viewer = transform.ParentUid;
        }
        return Vector3.Zero;
    }

    private static float FiniteNonnegative(float value) => float.IsFinite(value) ? Math.Max(0, value) : 0;
}
