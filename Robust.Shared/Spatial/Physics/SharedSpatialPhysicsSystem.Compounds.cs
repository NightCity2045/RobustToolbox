using System;
using System.Collections.Generic;
using System.Linq;
using Robust.Shared.GameObjects;
using Robust.Shared.Maths;

namespace Robust.Shared.Spatial.Physics;

public sealed partial class SharedSpatialPhysicsSystem
{
    internal sealed record CompoundMemberCapture(EntityUid Entity, RigidTransform3 RelativePose,
        SpatialBodyComponent? Body, ulong DefinitionRevision,
        SpatialGeometryComponent? Geometry, ulong GeometryRevision, bool GeometryAvailable);

    private readonly record struct CompoundBodyOverride(SpatialBodyDefinition Definition, ulong Revision);

    /// <summary>Prepared derived membership; canonical membership remains the Transform hierarchy.</summary>
    internal sealed record CompoundCapture(EntityUid Root, EntityUid[] ExpectedMembers, CompoundMemberCapture[] Members,
        SpatialBackendBodyDefinition? Definition);

    private EntityUid? FindCompoundRoot(EntityUid child)
        => SpatialPhysicsOwnership.FindCompoundRoot(EntityManager, child);

    private EntityUid? ResolvePhysicalOwner(EntityUid entity,
        IReadOnlyDictionary<EntityUid, CompoundBodyOverride>? overrides = null)
    {
        if (TerminatingOrDeleted(entity)) return null;
        while (true)
        {
            if (TryComp<SpatialBodyComponent>(entity, out var body))
            {
                var definition = overrides != null && overrides.TryGetValue(entity, out var changed)
                    ? changed.Definition : body.HasDefinition ? body.Definition : null;
                if (definition == null || body.LifeStage >= ComponentLifeStage.Stopping) return null;
                if (definition.Owner != SpatialMotionOwner.AnchoredCompoundChild) return entity;
            }
            else return HasComp<SpatialGeometryComponent>(entity) ? entity : null;
            var ancestor = Transform(entity).ParentUid;
            while (ancestor.IsValid() && !HasComp<SpatialBodyComponent>(ancestor) && !HasComp<SpatialGeometryComponent>(ancestor))
                ancestor = Transform(ancestor).ParentUid;
            if (!ancestor.IsValid() || TerminatingOrDeleted(ancestor)) return null;
            entity = ancestor;
        }
    }

    private bool TryResolveBodyBinding(EntityUid entity, out Binding binding, out RigidTransform3 relative)
    {
        relative = RigidTransform3.Identity;
        if (ResolvePhysicalOwner(entity) is { } root && _bindings.TryGetValue(root, out binding!))
        {
            relative = CompoundRelativePose(entity, root);
            return true;
        }
        binding = default!;
        return false;
    }

    private RigidTransform3 CompoundRelativePose(EntityUid child, EntityUid root)
    {
        var relative = RigidTransform3.Identity;
        while (child != root)
        {
            if (!child.IsValid()) throw new InvalidOperationException("A compound member left its root hierarchy.");
            var transform = Transform(child);
            relative = transform.LocalPose.Compose(relative);
            child = transform.ParentUid;
        }
        return relative;
    }

    private IEnumerable<EntityUid> CompoundChildren(EntityUid root,
        IReadOnlyDictionary<EntityUid, CompoundBodyOverride>? overrides = null)
    {
        var pending = new Stack<EntityUid>();
        foreach (var child in Transform(root)._children) pending.Push(child);
        while (pending.TryPop(out var uid))
        {
            if (TerminatingOrDeleted(uid) || _containers.IsEntityOrParentInContainer(uid)) continue;
            if (TryComp<SpatialBodyComponent>(uid, out var body))
            {
                // An independently moving passenger/cargo body owns its own
                // subtree of anchored parts. It never enters the vehicle body.
                var definition = overrides != null && overrides.TryGetValue(uid, out var changed)
                    ? changed.Definition : body.HasDefinition ? body.Definition : null;
                if (definition?.Owner != SpatialMotionOwner.AnchoredCompoundChild) continue;
                if (body.LifeStage >= ComponentLifeStage.Stopping) continue;
                yield return uid;
            }
            else if (HasComp<SpatialGeometryComponent>(uid))
                continue; // An implicit static geometry frame owns its own subtree.
            foreach (var child in Transform(uid)._children) pending.Push(child);
        }
    }

    private CompoundCapture CaptureCompound(EntityUid root, SpatialBodyDefinition rootDefinition, ulong rootRevision,
        IReadOnlyDictionary<EntityUid, SpatialGeometrySnapshot>? geometryOverrides = null,
        IReadOnlyDictionary<EntityUid, CompoundBodyOverride>? bodyOverrides = null)
    {
        var members = new List<CompoundMemberCapture>();
        var children = new List<SpatialCompoundMember>();
        var own = Capture(root, rootDefinition, rootRevision, RigidTransform3.Identity);
        foreach (var uid in CompoundChildren(root, bodyOverrides).Order())
        {
            if (rootDefinition.Owner == SpatialMotionOwner.Motor)
                throw new InvalidOperationException("A capsule motor cannot own structural compound children.");
            var body = Comp<SpatialBodyComponent>(uid);
            var relative = CompoundRelativePose(uid, root);
            var changed = bodyOverrides != null && bodyOverrides.TryGetValue(uid, out var next)
                ? next : new CompoundBodyOverride(body.Definition, body.DefinitionRevision);
            var definition = Capture(uid, changed.Definition, changed.Revision, relative);
            if (definition != null) children.Add(new SpatialCompoundMember(uid, relative, definition));
        }
        if (own == null && children.Count != 0)
            throw new InvalidOperationException("A compound root requires its own nonempty physical definition.");
        var combined = own == null || children.Count == 0 ? own : SpatialCompoundLayout.Compose(root, own, children);
        return new CompoundCapture(root, CompoundChildren(root).Append(root).ToArray(), members.ToArray(), combined);

        SpatialBackendBodyDefinition? Capture(EntityUid uid, SpatialBodyDefinition definition, ulong revision, RigidTransform3 relative)
        {
            TryComp<SpatialBodyComponent>(uid, out var body);
            TryComp<SpatialGeometryComponent>(uid, out var geometry);
            SpatialGeometrySnapshot? snapshot = null;
            geometryOverrides?.TryGetValue(uid, out snapshot);
            var available = geometry != null && geometry.LifeStage < ComponentLifeStage.Stopping;
            members.Add(new CompoundMemberCapture(uid, relative, body, body?.DefinitionRevision ?? 0,
                geometry, geometry?.Revision ?? 0, available));
            if (definition.UseGeometry && !available)
            {
                if (uid == root) throw new InvalidOperationException("The root body's canonical geometry is unavailable.");
                return null;
            }
            var backend = BuildBackendDefinition(uid, definition, revision, out _, snapshot);
            return backend;
        }
    }

    private bool CompoundStillCurrent(CompoundCapture capture)
    {
        if (TerminatingOrDeleted(capture.Root)) return false;
        var current = CompoundChildren(capture.Root).Append(capture.Root).ToHashSet();
        if (!current.SetEquals(capture.ExpectedMembers)) return false;
        foreach (var member in capture.Members)
        {
            if (TerminatingOrDeleted(member.Entity)
                || !CompoundRelativePose(member.Entity, capture.Root).Equals(member.RelativePose)) return false;
            TryComp<SpatialBodyComponent>(member.Entity, out var body);
            TryComp<SpatialGeometryComponent>(member.Entity, out var geometry);
            if (!ReferenceEquals(body, member.Body) || (body?.DefinitionRevision ?? 0) != member.DefinitionRevision
                || !ReferenceEquals(geometry, member.Geometry) || (geometry?.Revision ?? 0) != member.GeometryRevision
                || (geometry != null && geometry.LifeStage < ComponentLifeStage.Stopping) != member.GeometryAvailable) return false;
        }
        return true;
    }

    private static CompoundCapture WithCommittedCompoundVersions(CompoundCapture capture)
        => capture with
        {
            ExpectedMembers = capture.Members.Select(member => member.Entity).ToArray(),
            Members = capture.Members.Select(member => member with
            {
                DefinitionRevision = member.Body?.DefinitionRevision ?? 0,
                GeometryRevision = member.Geometry?.Revision ?? 0
            }).ToArray()
        };
}
