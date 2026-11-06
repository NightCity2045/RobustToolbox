using System;
using System.Collections.Generic;
using Robust.Shared.GameObjects;
using Robust.Shared.Map.Components;
using Robust.Shared.Physics.Components;

namespace Robust.Shared.Spatial.Physics;

/// <summary>Read-only preflight for physical profile and exclusive solver ownership.</summary>
internal static class SpatialPhysicsOwnership
{
    // Core transform/legacy-physics systems also run in minimal simulations that
    // do not register the spatial profile. Typed component queries require the
    // registration to exist even when the entity does not have the component.
    private static bool HasRegistered<T>(IEntityManager entities, EntityUid uid) where T : Component, new()
        => entities.ComponentFactory.TryGetRegistration<T>(out _) && entities.HasComponent<T>(uid);

    internal static EntityUid? GetMap(IEntityManager entities, EntityUid uid)
    {
        EntityUid? map = null;
        while (uid.IsValid())
        {
            if (HasRegistered<MapComponent>(entities, uid))
                map = uid;
            uid = entities.GetComponent<TransformComponent>(uid).ParentUid;
        }
        return map;
    }

    internal static void ValidateLegacyBody(IEntityManager entities, EntityUid uid)
    {
        if (HasRegistered<SpatialBodyComponent>(entities, uid)
            || GetMap(entities, uid) is { } map && HasRegistered<SpatialPhysicsWorldComponent>(entities, map))
            throw new InvalidOperationException("Legacy planar physics cannot own a spatial body or run inside a spatial physics world.");
    }

    internal static void ValidateReparent(IEntityManager entities, EntityUid root, EntityUid parent)
    {
        var map = GetMap(entities, parent);
        // Nullspace is a dormant state, not a second physical simulation.
        if (map == null)
            return;
        var spatial = HasRegistered<SpatialPhysicsWorldComponent>(entities, map.Value);
        ValidateSubtree(entities, root, spatial);
        if (!spatial)
            return;
        var physicalParent = false;
        for (var ancestor = parent; ancestor.IsValid(); ancestor = entities.GetComponent<TransformComponent>(ancestor).ParentUid)
            physicalParent |= HasRegistered<SpatialBodyComponent>(entities, ancestor);
        var pending = new Stack<(EntityUid Entity, bool PhysicalParent)>();
        pending.Push((root, physicalParent));
        while (pending.TryPop(out var item))
        {
            var (uid, owned) = item;
            SpatialBodyComponent? body = null;
            var hasBody = HasRegistered<SpatialBodyComponent>(entities, uid)
                && entities.TryGetComponent(uid, out body) && body.HasDefinition;
            if (owned && (hasBody && body!.Definition.Owner == SpatialMotionOwner.Static
                || !hasBody && HasRegistered<SpatialGeometryComponent>(entities, uid)))
                throw new InvalidOperationException("Attached structural geometry requires compound ownership, not an independent static body.");
            using var children = entities.GetComponent<TransformComponent>(uid).ChildEnumerator;
            while (children.MoveNext(out var child))
                pending.Push((child, owned || hasBody));
        }
    }

    internal static void ValidateSubtree(IEntityManager entities, EntityUid root, bool spatialDestination)
    {
        var pending = new Stack<EntityUid>();
        pending.Push(root);
        while (pending.TryPop(out var uid))
        {
            if (spatialDestination && HasRegistered<PhysicsComponent>(entities, uid))
                throw new InvalidOperationException("Migrate planar physics before moving the hierarchy into a spatial physics world.");
            if (!spatialDestination && HasRegistered<SpatialBodyComponent>(entities, uid))
                throw new InvalidOperationException("A spatial body requires a spatial physics destination world.");
            using var children = entities.GetComponent<TransformComponent>(uid).ChildEnumerator;
            while (children.MoveNext(out var child))
                pending.Push(child);
        }
    }
}
