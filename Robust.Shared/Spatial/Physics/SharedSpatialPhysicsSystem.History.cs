using System;
using System.Collections.Generic;
using System.Linq;
using Robust.Shared.GameObjects;

namespace Robust.Shared.Spatial.Physics;

[ByRefEvent]
internal readonly struct SpatialPhysicsStepCompleted;

internal sealed record SpatialHistoryBody(EntityUid Entity, SpatialBackendBodyDefinition Definition,
    SpatialBackendState State, ulong DefinitionRevision, EntityUid[] GeometryFrames,
    KeyValuePair<EntityUid, ulong>[] AnchoredMembers);

public sealed partial class SharedSpatialPhysicsSystem
{
    // Read-only capture at the completed physics boundary. The history never
    // rewinds canonical entities or the live solver to answer past queries.
    internal IEnumerable<SpatialHistoryBody> CaptureHistoryBodies(EntityUid world)
    {
        RequireServer();
        if (!_worlds.TryGetValue(world, out var backend)) yield break;
        foreach (var binding in _bindings.Values)
        {
            if (binding.World != world || TerminatingOrDeleted(binding.Entity)) continue;
            var state = new SpatialBackendState(_transforms.GetWorldPose(binding.Entity),
                TryComp<SpatialBodyComponent>(binding.Entity, out var body) ? body.LinearVelocity : default,
                body?.AngularVelocity ?? default);
            var frames = binding.Compound?.Members.Where(member => member.GeometryAvailable && member.Geometry != null)
                .Select(member => member.Entity).ToArray()
                ?? (binding.Geometry ? new[] { binding.Entity } : Array.Empty<EntityUid>());
            var members = binding.Compound?.Members.Where(member => member.Entity != binding.Entity && member.Body != null)
                .Select(member => new KeyValuePair<EntityUid, ulong>(member.Entity, member.DefinitionRevision)).ToArray()
                ?? Array.Empty<KeyValuePair<EntityUid, ulong>>();
            yield return new SpatialHistoryBody(binding.Entity, backend.GetDefinition(binding.Body), state,
                binding.DefinitionRevision, frames, members);
        }
    }
}
