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

    /// <summary>
    /// Whether this client is currently replaying input for an explicitly selected,
    /// active motor. Shared gameplay can skip remote motors without depending on
    /// client systems. False on the authoritative server and outside prediction.
    /// This is write eligibility, not collision readiness or permission for a
    /// gameplay action; unavailable geometry can still prevent motion or resize.
    /// </summary>
    public bool IsPredictingMotor(GameObjects.EntityUid entity)
        => MotorPredictionDriver?.CanPredict(entity) == true;

    private void RequireMotorAuthority(GameObjects.EntityUid entity)
    {
        if (!_net.IsServer && !IsPredictingMotor(entity))
            throw new InvalidOperationException("A client motor write requires an explicitly selected motor during input prediction.");
    }
}
