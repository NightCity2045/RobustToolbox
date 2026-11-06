using System;
using BepuPhysics;
using BepuPhysics.Collidables;

namespace Robust.Shared.Spatial.Physics;

internal sealed partial class BepuSpatialPhysicsBackend
{
    /// <summary>
    /// Owns validated shapes/inertia outside the broadphase. Preparation cannot
    /// wake a live body, collide or emit contacts. Commit consumes it exactly once.
    /// </summary>
    internal sealed class PreparedBody(BepuSpatialPhysicsBackend owner, Entry entry) : IDisposable
    {
        private Entry? _entry = entry;

        public SpatialBackendBodyId Commit(SpatialBackendState state)
        {
            owner.ThrowIfDisposed();
            var prepared = _entry ?? throw new InvalidOperationException("Prepared body has already been consumed or disposed.");
            var id = owner.CommitBody(prepared, state);
            _entry = null;
            return id;
        }

        public void ValidateState(SpatialBackendState state)
        {
            owner.ThrowIfDisposed();
            var prepared = _entry ?? throw new InvalidOperationException("Prepared body has already been consumed or disposed.");
            BepuSpatialPhysicsBackend.ValidateState(state, prepared.Definition);
            ValidateSolverState(ToSolverPose(state.Pose, prepared.Center), ToSolverVelocity(state, prepared.Center));
        }

        public void Dispose()
        {
            if (_entry == null)
                return;
            if (!owner._disposed)
                owner._simulation.Shapes.RecursivelyRemoveAndDispose(_entry.Shape, owner._pool);
            _entry = null;
        }
    }

    private SpatialBackendBodyId CommitBody(Entry entry, SpatialBackendState state)
    {
        ValidateState(state, entry.Definition);
        var pose = ToSolverPose(state.Pose, entry.Center);
        var velocity = ToSolverVelocity(state, entry.Center);
        ValidateSolverState(pose, velocity);
        var id = new SpatialBackendBodyId(checked(++_nextId));
        entry.Id = id;
        var added = false;
        try
        {
            if (entry.Definition.Motion == SpatialBackendMotion.Static)
            {
                entry.Static = _simulation.Statics.Add(new StaticDescription(pose, entry.Shape));
                added = true;
                _statics.Add(entry.Static.Value, entry);
            }
            else
            {
                var collidable = new CollidableDescription(entry.Shape, 1f, ContinuousDetection.Continuous(1e-6f, 1e-6f));
                var description = entry.Definition.Motion == SpatialBackendMotion.Dynamic
                    ? BodyDescription.CreateDynamic(pose, velocity, entry.DynamicInertia, collidable, 0.01f)
                    : BodyDescription.CreateKinematic(pose, velocity, collidable, 0.01f);
                entry.Body = _simulation.Bodies.Add(description);
                added = true;
                _bodies.Add(entry.Body.Value, entry);
            }
            _entries.Add(id, entry);
            return id;
        }
        catch
        {
            if (added)
            {
                if (entry.Definition.Motion == SpatialBackendMotion.Static)
                {
                    _statics.Remove(entry.Static.Value);
                    _simulation.Statics.Remove(entry.Static);
                }
                else
                {
                    _bodies.Remove(entry.Body.Value);
                    _simulation.Bodies.Remove(entry.Body);
                }
            }
            throw; // The unconsumed PreparedBody still owns and disposes its shapes.
        }
    }
}
