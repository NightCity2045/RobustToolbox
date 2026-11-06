using System;
using System.Collections.Generic;
using System.Linq;
using Robust.Shared.Map;
using Robust.Shared.Map.Components;
using Robust.Shared.Maths;
using Robust.Shared.Spatial;
using Robust.Shared.Spatial.Physics;

namespace Robust.Shared.GameObjects;

public abstract partial class SharedTransformSystem
{
    internal sealed record SpatialTransferNode(Entity<TransformComponent, MetaDataComponent> Entity,
        EntityUid Parent, RigidTransform3 LocalPose, EntityUid? Grid, TimeSpan? PauseTime);

    /// <summary>Short-lived read-only preflight; it is never an authoritative hierarchy.</summary>
    internal sealed record SpatialHierarchyTransfer(EntityUid Source, EntityUid Destination,
        MapId OldMapId, MapId NewMapId, bool Paused, RigidTransform3 Mapping,
        IReadOnlyDictionary<EntityUid, RigidTransform3> Roots, IReadOnlyList<SpatialTransferNode> Nodes);

    internal SpatialHierarchyTransfer PrepareSpatialHierarchyTransfer(IReadOnlyCollection<EntityUid> roots,
        EntityUid destination, RigidTransform3 mapping)
    {
        ValidatePose(mapping);
        if (roots.Count == 0 || roots.Count != roots.Distinct().Count())
            throw new ArgumentException("A transfer needs distinct hierarchy roots.", nameof(roots));
        if (TerminatingOrDeleted(destination) || !HasComp<SpatialWorldComponent>(destination)
            || !TryComp<MapComponent>(destination, out var targetMap))
            throw new InvalidOperationException("Transfer destination must be a running spatial map.");
        var targetPose = GetWorldPose(destination);
        EntityUid? source = null;
        var visited = new HashSet<EntityUid>();
        var nodes = new List<SpatialTransferNode>();
        var poses = new Dictionary<EntityUid, RigidTransform3>();
        foreach (var root in roots)
        {
            if (TerminatingOrDeleted(root) || HasComp<MapComponent>(root) || _container.IsEntityOrParentInContainer(root))
                throw new InvalidOperationException("A map or contained entity cannot be a transfer root.");
            var xform = Transform(root);
            source ??= xform.MapUid;
            if (source == null || source == destination || xform.MapUid != source || !HasComp<SpatialWorldComponent>(source.Value))
                throw new InvalidOperationException("Roots must share a source spatial world distinct from the destination.");
            SpatialPhysicsOwnership.ValidateReparent(EntityManager, root, destination);
            var pose = targetPose.ToLocal(mapping.Compose(GetWorldPose(root)));
            if (xform.Anchored || xform.NoLocalRotation && !pose.Rotation.Equals(xform.LocalPose.Rotation))
                throw new InvalidOperationException("Transfer cannot override a root's transform movement restrictions.");
            poses.Add(root, pose);
            var pending = new Stack<EntityUid>();
            pending.Push(root);
            while (pending.TryPop(out var uid))
            {
                var transform = Transform(uid);
                var meta = MetaData(uid);
                if (!visited.Add(uid) || !transform.Running || meta.EntityLifeStage >= EntityLifeStage.Terminating
                    || transform.MapUid != source || HasComp<MapGridComponent>(uid))
                    throw new InvalidOperationException("Transfer requires disjoint running spatial frame hierarchies, without legacy grids.");
                // Nonphysical container contents also need representable world poses.
                ValidatePose(mapping.Compose(GetWorldPose(uid)));
                nodes.Add(new SpatialTransferNode((uid, transform, meta), transform.ParentUid, transform.LocalPose,
                    transform.GridUid, meta.PauseTime));
                foreach (var child in transform._children) pending.Push(child);
            }
        }
        return new SpatialHierarchyTransfer(source!.Value, destination, Transform(source.Value).MapID,
            targetMap.MapId, _map.IsPaused(targetMap.MapId), mapping, poses, nodes);
    }

    internal void ValidateSpatialHierarchyTransfer(SpatialHierarchyTransfer transfer)
    {
        if (TerminatingOrDeleted(transfer.Destination) || Transform(transfer.Destination).MapID != transfer.NewMapId
            || _map.IsPaused(transfer.NewMapId) != transfer.Paused)
            throw new InvalidOperationException("Transfer destination changed during preparation.");
        var members = transfer.Nodes.Select(n => n.Entity.Owner).ToHashSet();
        foreach (var node in transfer.Nodes)
        {
            var (uid, xform, meta) = node.Entity;
            if (TerminatingOrDeleted(uid) || !ReferenceEquals(Transform(uid), xform) || !ReferenceEquals(MetaData(uid), meta)
                || xform.ParentUid != node.Parent || !xform.LocalPose.Equals(node.LocalPose) || xform.MapUid != transfer.Source
                || meta.PauseTime != node.PauseTime || xform._children.Any(child => !members.Contains(child)))
                throw new InvalidOperationException("Transfer hierarchy changed during preparation.");
        }
    }

    /// <summary>All hierarchy/map/pause state and solver bindings precede notifications.</summary>
    internal void CommitSpatialHierarchyTransfer(SpatialHierarchyTransfer transfer, Action installPhysics)
    {
        ValidateSpatialHierarchyTransfer(transfer);
        foreach (var node in transfer.Nodes)
        {
            var (uid, xform, meta) = node.Entity;
            if (transfer.Roots.TryGetValue(uid, out var local))
            {
                Transform(xform.ParentUid)._children.Remove(uid);
                Transform(transfer.Destination)._children.Add(uid);
                xform._parent = transfer.Destination;
                xform._localPosition3 = local.Position;
                xform._localRotation3 = local.Rotation;
                xform.MatricesDirty = true;
                DirtyFields(uid, xform, meta, nameof(TransformComponent.LocalPosition),
                    nameof(TransformComponent.LocalRotation), nameof(TransformComponent.ParentUid));
            }
            xform.MapUid = transfer.Destination;
            xform.MapID = transfer.NewMapId;
            xform._gridUid = null;
        }
        _metaData.CommitSpatialPauseStates(transfer.Nodes.Select(n => new Entity<MetaDataComponent>(n.Entity.Owner, n.Entity.Comp2)).ToArray(),
            transfer.Paused, installPhysics);
        foreach (var node in transfer.Nodes)
        {
            var (uid, xform, meta) = node.Entity;
            if (TerminatingOrDeleted(uid)) continue;
            if ((meta.Flags & MetaDataFlags.ExtraTransformEvents) != 0)
            {
                var mapEvent = new MapUidChangedEvent(node.Entity, transfer.Source, transfer.OldMapId);
                RaiseLocalEvent(uid, ref mapEvent);
                if (node.Grid != null && !TerminatingOrDeleted(uid))
                {
                    var gridEvent = new GridUidChangedEvent(node.Entity, node.Grid);
                    RaiseLocalEvent(uid, ref gridEvent);
                }
            }
            if (transfer.Roots.ContainsKey(uid) && !TerminatingOrDeleted(uid))
                RaiseMoveEvent(node.Entity, node.Parent, node.LocalPose, transfer.Source, checkTraversal: false);
        }
    }
}
