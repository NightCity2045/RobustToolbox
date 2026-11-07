using System;
using System.Numerics;
using Robust.Shared.Maths;

namespace Robust.Shared.Spatial.Physics;

internal sealed partial class BepuSpatialPhysicsBackend
{
    private MotorCoverage? _motorCoverage;

    /// <summary>
    /// Certifies a speculative motor step against received collision coverage.
    /// Does not invent walls or supports. The caller must discard the entire
    /// speculative solver when Available is false, before publishing any state.
    /// </summary>
    internal MotorCoverage BeginMotorCoverage(SpatialBackendBodyId body, Box3 bounds)
    {
        ThrowIfDisposed();
        var entry = _entries[body];
        if (_motorCoverage != null || !bounds.IsValid || !entry.Definition.LockRotation
            || entry.Definition.Parts.Count != 1 || entry.Definition.Parts[0].Shape is not SpatialCapsule)
            throw new InvalidOperationException("Coverage requires one rotation-locked capsule and an exclusive scope.");
        var coverage = new MotorCoverage(this, entry, bounds);
        _motorCoverage = coverage;
        coverage.Observe();
        return coverage;
    }

    internal sealed class MotorCoverage : IDisposable
    {
        private readonly BepuSpatialPhysicsBackend _owner;
        private readonly Entry _entry;
        private readonly Box3 _bounds;
        public bool Available { get; private set; } = true;

        internal MotorCoverage(BepuSpatialPhysicsBackend owner, Entry entry, Box3 bounds)
        { _owner = owner; _entry = entry; _bounds = bounds; }

        internal void Check(Box3 query)
            => Available &= _bounds.Contains(query.Min) && _bounds.Contains(query.Max);

        internal void Observe()
        {
            var center = _owner.SolverPose(_entry).Position;
            var extent = new Vector3(_entry.Radius);
            // Locked rotation makes each substep's complete swept volume lie
            // inside the convex hull of these endpoint spheres. Observing every
            // solver substep also catches leaving and returning in one tick.
            Available &= SpatialBackendBodyDefinition.Finite(center)
                && _bounds.Contains(center - extent) && _bounds.Contains(center + extent);
        }

        public void Dispose()
        {
            if (ReferenceEquals(_owner._motorCoverage, this)) _owner._motorCoverage = null;
        }
    }
}
