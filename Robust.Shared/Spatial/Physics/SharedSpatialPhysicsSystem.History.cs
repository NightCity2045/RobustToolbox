using System.Collections.Generic;
using Robust.Shared.GameObjects;

namespace Robust.Shared.Spatial.Physics;

[ByRefEvent]
internal readonly struct SpatialPhysicsStepCompleted;

internal sealed record SpatialHistoryBody(EntityUid Entity, SpatialBackendBodyDefinition Definition,
    SpatialBackendState State, ulong DefinitionRevision, bool Geometry);

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
            yield return new SpatialHistoryBody(binding.Entity, backend.GetDefinition(binding.Body), state,
                binding.DefinitionRevision, binding.Geometry);
        }
    }
}
