using System;
using System.Collections.Generic;
using Robust.Shared.GameObjects;
using Robust.Shared.Maths;

namespace Robust.Shared.Spatial.Physics;

/// <summary>Derived placement in a root body's coordinates, never a canonical entity pose.</summary>
internal readonly record struct SpatialCompoundMember(EntityUid Entity, RigidTransform3 RelativePose,
    SpatialBackendBodyDefinition Definition);

internal static class SpatialCompoundLayout
{
    /// <summary>
    /// Flatten rigidly attached members while preserving each part's semantic
    /// identity, filter and physical mass contribution. The root owns motion,
    /// solver material and rotation policy for the resulting single body.
    /// </summary>
    internal static SpatialBackendBodyDefinition Compose(EntityUid root, SpatialBackendBodyDefinition definition,
        IEnumerable<SpatialCompoundMember> children)
    {
        ArgumentNullException.ThrowIfNull(definition);
        ArgumentNullException.ThrowIfNull(children);
        if (!root.IsValid()) throw new ArgumentException("A compound requires a valid root entity.");
        var seen = new HashSet<EntityUid> { root };
        var parts = new List<SpatialBackendPart>();
        double mass = 0;
        Add(new SpatialCompoundMember(root, RigidTransform3.Identity, definition));
        foreach (var child in children)
        {
            if (!child.Entity.IsValid() || !seen.Add(child.Entity) || !child.RelativePose.IsValid || child.Definition == null)
                throw new ArgumentException("Compound members require unique valid entities, definitions and relative poses.");
            Add(child);
        }
        return new SpatialBackendBodyDefinition(definition.Motion, parts, (float) mass, definition.Material,
            definition.CollisionLayer, definition.CollisionMask, definition.LockRotation);

        void Add(SpatialCompoundMember member)
        {
            mass += member.Definition.Mass;
            if (mass > float.MaxValue) throw new ArgumentException("Compound mass exceeds the finite physical range.");
            double totalWeight = 0;
            foreach (var part in member.Definition.Parts) totalWeight += part.MassWeight;
            foreach (var part in member.Definition.Parts)
            {
                var weight = (float) (member.Definition.Mass * ((double) part.MassWeight / totalWeight));
                if (!float.IsFinite(weight) || weight <= 0 || !float.IsFinite(1 / weight))
                    throw new ArgumentException("Compound part mass exceeds the representable physical range.");
                parts.Add(part with
                {
                    LocalPose = member.RelativePose.Compose(part.LocalPose),
                    Entity = part.Entity ?? member.Entity,
                    MassWeight = weight,
                    CollisionLayer = part.CollisionLayer ?? member.Definition.CollisionLayer,
                    CollisionMask = part.CollisionMask ?? member.Definition.CollisionMask
                });
            }
        }
    }
}
