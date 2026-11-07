using System;

namespace Robust.Shared.Spatial.Physics;

/// <summary>Trusted client driver; gameplay cannot install its own solver.</summary>
internal interface ISpatialMotorPredictionDriver
{
    bool CanPredict(GameObjects.EntityUid entity);
    bool TryResize(GameObjects.EntityUid entity, float radius, float length, ulong revision);
}

public sealed partial class SharedSpatialPhysicsSystem
{
    internal ISpatialMotorPredictionDriver? MotorPredictionDriver;

    private void RequireMotorAuthority(GameObjects.EntityUid entity)
    {
        if (!_net.IsServer && MotorPredictionDriver?.CanPredict(entity) != true)
            throw new InvalidOperationException("A client motor write requires an explicitly selected motor during input prediction.");
    }
}
