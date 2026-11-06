using System;
using System.Numerics;
using Robust.Shared.GameObjects;

namespace Robust.Shared.Spatial.Physics;

public sealed partial class SharedSpatialPhysicsSystem
{
    internal PreparedGeometryBody? PrepareGeometryBody(EntityUid uid, EntityUid world, SpatialGeometrySnapshot after, bool unchanged)
    {
        RequireServer();
        if (!_worlds.TryGetValue(world, out var backend))
        {
            if (HasComp<SpatialPhysicsWorldComponent>(world))
                throw new InvalidOperationException("The spatial physics world is not ready for an edit.");
            return null;
        }
        TryComp<SpatialBodyComponent>(uid, out var body);
        var definition = body?.Definition ?? new SpatialBodyDefinition(SpatialMotionOwner.Static, useGeometry: true);
        ValidateGeometryOwnership(uid, definition);
        var revision = body?.DefinitionRevision ?? 0;
        var replacement = unchanged ? null : BuildBackendDefinition(uid, definition, revision, out _, after);
        var prepared = replacement == null ? null : backend.PrepareBody(replacement);
        var result = new PreparedGeometryBody(this, uid, world, backend, body, revision, after.Revision, prepared, unchanged);
        try
        {
            result.Validate();
            return result;
        }
        catch
        {
            result.Dispose();
            throw;
        }
    }

    /// <summary>Transaction-owned shapes. No public solver handle or canonical mutation during preparation.</summary>
    internal sealed class PreparedGeometryBody(SharedSpatialPhysicsSystem owner, EntityUid entity, EntityUid world,
        BepuSpatialPhysicsBackend backend, SpatialBodyComponent? expectedBody, ulong definitionRevision,
        ulong geometryRevision, BepuSpatialPhysicsBackend.PreparedBody? prepared, bool unchanged) : IDisposable
    {
        private Binding? _replacement;
        private SpatialBackendState _state;
        private bool _insert;
        private bool _paused;
        private bool _validated;
        private bool _installed;
        private bool _disposed;

        internal void Validate()
        {
            if (_disposed || _installed || owner.TerminatingOrDeleted(entity)
                || SpatialPhysicsOwnership.GetMap(owner.EntityManager, entity) != world
                || !owner._worlds.TryGetValue(world, out var active) || active != backend)
                throw new InvalidOperationException("Prepared geometry lost its world or owner.");
            owner.TryComp<SpatialBodyComponent>(entity, out var body);
            if (!ReferenceEquals(body, expectedBody) || body != null && (!body.Running || body.DefinitionRevision != definitionRevision))
                throw new InvalidOperationException("The physical definition changed after geometry preparation.");
            var definition = body?.Definition ?? new SpatialBodyDefinition(SpatialMotionOwner.Static, useGeometry: true);
            var linear = body?.LinearVelocity ?? Vector3.Zero;
            var angular = body?.AngularVelocity ?? Vector3.Zero;
            owner.ValidateDefinition(entity, definition, linear, angular);
            owner.ValidateGeometryOwnership(entity, definition);
            var canonical = new SpatialBackendState(owner._transforms.GetWorldPose(entity), linear, angular);
            prepared?.ValidateState(canonical);
            _paused = body != null && owner.MetaData(entity).EntityPaused;
            _state = _paused ? canonical with { LinearVelocity = Vector3.Zero, AngularVelocity = Vector3.Zero } : canonical;
            _insert = !owner._containers.IsEntityOrParentInContainer(entity);
            _validated = true;
        }

        internal void CommitNewBody()
        {
            if (!_validated || _disposed || _installed || _replacement != null)
                throw new InvalidOperationException("Geometry body has not been validated for commit.");
            if (!_insert || prepared == null)
                return;
            var id = prepared.Commit(_state);
            _replacement = new Binding(entity, world, id, true, geometryRevision, _paused, definitionRevision);
            if (_paused && expectedBody != null && IsDynamicOwner(expectedBody.Definition.Owner))
                backend.SetPaused(id, true);
        }

        internal void Install()
        {
            if (unchanged)
            {
                if (owner._bindings.TryGetValue(entity, out var current))
                    owner._bindings[entity] = current with { GeometryRevision = geometryRevision };
                _installed = true;
                return;
            }
            owner.RemoveBinding(entity);
            if (_replacement != null)
                owner._bindings.Add(entity, _replacement);
            _installed = true;
        }

        public void Dispose()
        {
            if (_disposed)
                return;
            if (!_installed && _replacement != null && owner._worlds.TryGetValue(world, out var active) && active == backend)
                backend.RemoveBody(_replacement.Body);
            prepared?.Dispose();
            _disposed = true;
        }
    }
}
