using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using Robust.Shared.Containers;
using Robust.Shared.GameObjects;
using Robust.Shared.GameStates;
using Robust.Shared.IoC;
using Robust.Shared.Map.Components;
using Robust.Shared.Network;
using Robust.Shared.Physics.Components;

namespace Robust.Shared.Spatial.Physics;

/// <summary>
/// Server integration of spatial bodies with the existing transform hierarchy.
/// Backend state is committed into Transform; components own canonical velocities.
/// Clients currently receive authoritative state, pending the prediction/motor path.
/// </summary>
public sealed partial class SharedSpatialPhysicsSystem : EntitySystem
{
    [Dependency] private INetManager _net = default!;
    [Dependency] private SharedTransformSystem _transforms = default!;
    [Dependency] private SharedGeometrySystem _geometry = default!;
    [Dependency] private SharedContainerSystem _containers = default!;

    private readonly Dictionary<EntityUid, BepuSpatialPhysicsBackend> _worlds = new();
    private readonly Dictionary<EntityUid, Binding> _bindings = new();
    private bool _committing;

    private sealed record Binding(EntityUid Entity, EntityUid World, SpatialBackendBodyId Body,
        bool Geometry, ulong GeometryRevision, bool Paused);

    public override void Initialize()
    {
        base.Initialize();
        InitializeConstraints();
        SubscribeLocalEvent<SpatialPhysicsWorldComponent, ComponentStartup>(OnWorldStartup);
        SubscribeLocalEvent<SpatialPhysicsWorldComponent, ComponentShutdown>(OnWorldShutdown);
        SubscribeLocalEvent<SpatialPhysicsWorldComponent, ComponentGetState>(OnWorldGetState);
        SubscribeLocalEvent<SpatialPhysicsWorldComponent, ComponentHandleState>(OnWorldHandleState);
        SubscribeLocalEvent<SpatialBodyComponent, ComponentStartup>(OnBodyStartup);
        SubscribeLocalEvent<SpatialBodyComponent, ComponentShutdown>(OnBodyShutdown);
        SubscribeLocalEvent<SpatialBodyComponent, ComponentGetState>(OnBodyGetState);
        SubscribeLocalEvent<SpatialBodyComponent, ComponentHandleState>(OnBodyHandleState);
        SubscribeLocalEvent<SpatialBodyComponent, EntityPausedEvent>(OnBodyPaused);
        SubscribeLocalEvent<SpatialBodyComponent, EntityUnpausedEvent>(OnBodyUnpaused);
        SubscribeLocalEvent<SpatialGeometryComponent, ComponentStartup>(OnGeometryStartup);
        SubscribeLocalEvent<SpatialGeometryComponent, ComponentShutdown>(OnGeometryShutdown);
        SubscribeLocalEvent<EntGotInsertedIntoContainerMessage>(OnContainerInserted);
        SubscribeLocalEvent<EntGotRemovedFromContainerMessage>(OnContainerRemoved);
        _transforms.OnGlobalMoveEvent += OnMove;
    }

    public override void Shutdown()
    {
        _transforms.OnGlobalMoveEvent -= OnMove;
        foreach (var world in _worlds.Values)
            world.Dispose();
        _worlds.Clear();
        _bindings.Clear();
        _constraintBindings.Clear();
        _constraintComponents.Clear();
        _semanticContacts.Clear();
        base.Shutdown();
    }

    /// <summary>Enable the physical profile of an existing spatial map after validating its hierarchy.</summary>
    public void EnablePhysics(EntityUid world, SpatialPhysicsSettings? settings = null)
    {
        RequireServer();
        if (!HasComp<SpatialWorldComponent>(world) || !HasComp<MapComponent>(world))
            throw new InvalidOperationException("Physics requires an existing SpatialWorld map.");
        if (HasComp<SpatialPhysicsWorldComponent>(world))
            throw new InvalidOperationException("Spatial world physics is already enabled.");
        var value = settings ?? SpatialPhysicsSettings.Default;
        value.Validate();
        SpatialPhysicsOwnership.ValidateSubtree(EntityManager, world, true);
        EntityManager.AddComponent(world, new SpatialPhysicsWorldComponent { _settings = value });
    }

    public void ConfigureInitialBody(Entity<SpatialBodyComponent> body, SpatialBodyDefinition definition,
        Vector3 linearVelocity = default, Vector3 angularVelocity = default)
    {
        RequireServer();
        if (MetaData(body).EntityLifeStage != EntityLifeStage.PreInit)
            throw new InvalidOperationException("Initial body configuration requires an uninitialized entity.");
        ValidateDefinition(body, definition, linearVelocity, angularVelocity);
        var copy = definition.Copy();
        body.Comp._definition = copy;
        body.Comp._linearVelocity = linearVelocity;
        body.Comp._angularVelocity = angularVelocity;
    }

    /// <summary>Set world-space velocity at the entity origin, including for a dormant contained body.</summary>
    public void SetVelocity(Entity<SpatialBodyComponent> body, Vector3 linear, Vector3 angular)
    {
        RequireServer();
        ValidateDefinition(body, body.Comp._definition, linear, angular);
        if (_bindings.TryGetValue(body, out var binding) && !binding.Paused)
        {
            var backend = _worlds[binding.World];
            var state = backend.GetState(binding.Body);
            backend.SetState(binding.Body, new SpatialBackendState(state.Pose, linear, angular));
        }
        body.Comp._linearVelocity = linear;
        body.Comp._angularVelocity = angular;
        Dirty(body);
    }

    public override void Update(float frameTime)
    {
        if (!_net.IsServer || frameTime <= 0)
            return;
        SyncConstraints();
        foreach (var (world, backend) in _worlds)
        {
            if (MetaData(world).EntityPaused)
                continue;
            backend.Step(frameTime);
        }

        var poses = new Dictionary<EntityUid, Maths.RigidTransform3>();
        var velocities = new List<(EntityUid Entity, SpatialBodyComponent Component, SpatialBackendState State)>();
        foreach (var binding in _bindings.Values)
        {
            if (!_worlds.TryGetValue(binding.World, out var backend) || TerminatingOrDeleted(binding.Entity)
                || MetaData(binding.World).EntityPaused || !TryComp<SpatialBodyComponent>(binding.Entity, out var component)
                || component.Definition.Owner != SpatialMotionOwner.DynamicBody)
                continue;
            var state = backend.GetState(binding.Body);
            poses.Add(binding.Entity, state.Pose);
            if (!binding.Paused)
                velocities.Add((binding.Entity, component, state));
        }
        _committing = true;
        try
        {
            _transforms.CommitSpatialPoses(poses, () =>
            {
                foreach (var (uid, component, state) in velocities)
                {
                    if (component._linearVelocity != state.LinearVelocity || component._angularVelocity != state.AngularVelocity)
                    {
                        component._linearVelocity = state.LinearVelocity;
                        component._angularVelocity = state.AngularVelocity;
                        Dirty(uid, component);
                    }
                }
            });
        }
        finally { _committing = false; }
        PublishContacts();
    }

    private void OnWorldStartup(EntityUid uid, SpatialPhysicsWorldComponent component, ComponentStartup args)
    {
        component._settings.Validate();
        if (!HasComp<MapComponent>(uid) || !HasComp<SpatialWorldComponent>(uid))
            throw new InvalidOperationException("A physical profile belongs to a SpatialWorld map.");
        if (!_net.IsServer)
            return;
        SpatialPhysicsOwnership.ValidateSubtree(EntityManager, uid, true);
        var settings = component._settings;
        _worlds.Add(uid, new BepuSpatialPhysicsBackend(settings.Gravity, settings.Substeps, settings.SolverIterations));
        RefreshSubtree(uid);
    }

    private void OnWorldShutdown(EntityUid uid, SpatialPhysicsWorldComponent component, ComponentShutdown args)
    {
        ForgetWorldContacts(uid);
        if (!_worlds.Remove(uid, out var backend))
            return;
        foreach (var binding in _bindings.Values.Where(b => b.World == uid).ToArray())
            _bindings.Remove(binding.Entity);
        backend.Dispose();
    }

    private void OnBodyStartup(EntityUid uid, SpatialBodyComponent component, ComponentStartup args)
    {
        if (!_net.IsServer)
            return;
        ValidateDefinition(uid, component._definition, component._linearVelocity, component._angularVelocity);
        component._definition = component._definition.Copy();
        RefreshBinding(uid);
    }

    private void OnBodyShutdown(EntityUid uid, SpatialBodyComponent component, ComponentShutdown args)
    {
        RemoveConstraintsReferencing(uid);
        RemoveBinding(uid);
    }

    private void OnGeometryStartup(EntityUid uid, SpatialGeometryComponent component, ComponentStartup args)
    {
        if (_net.IsServer && !HasComp<SpatialBodyComponent>(uid))
            RefreshBinding(uid);
    }

    private void OnGeometryShutdown(EntityUid uid, SpatialGeometryComponent component, ComponentShutdown args)
    {
        RemoveConstraintsReferencing(uid);
        if (_bindings.TryGetValue(uid, out var binding) && binding.Geometry)
            RemoveBinding(uid);
    }

    private void OnMove(ref MoveEvent args)
    {
        if (!_net.IsServer || _committing)
            return;
        var root = args.Sender;
        // If the owner moved, independent bodies keep their solver world pose.
        // Contained bodies are dormant and continue to follow container transforms.
        var subtree = GetSubtree(root);
        _committing = true;
        try
        {
            foreach (var uid in subtree)
            {
                if (uid == root || !_bindings.TryGetValue(uid, out var binding) || TerminatingOrDeleted(uid)
                    || !TryComp<SpatialBodyComponent>(uid, out var body) || body.Definition.Owner != SpatialMotionOwner.DynamicBody
                    || _containers.IsEntityOrParentInContainer(uid) || !_worlds.TryGetValue(binding.World, out var backend))
                    continue;
                _transforms.SetWorldPose(binding.Entity, backend.GetState(binding.Body).Pose);
            }
        }
        finally { _committing = false; }
        foreach (var uid in subtree)
            if (EntityManager.EntityExists(uid))
                RefreshBinding(uid);
    }

    private void OnContainerInserted(EntGotInsertedIntoContainerMessage args) => RefreshSubtree(args.Entity);
    private void OnContainerRemoved(EntGotRemovedFromContainerMessage args) => RefreshSubtree(args.Entity);
    private void OnBodyPaused(EntityUid uid, SpatialBodyComponent component, ref EntityPausedEvent args)
    {
        if (_net.IsServer)
            RefreshBinding(uid);
    }
    private void OnBodyUnpaused(EntityUid uid, SpatialBodyComponent component, ref EntityUnpausedEvent args)
    {
        if (_net.IsServer)
            RefreshBinding(uid);
    }

    private void RefreshSubtree(EntityUid root)
    {
        if (!_net.IsServer || !EntityManager.EntityExists(root))
            return;
        foreach (var uid in GetSubtree(root))
            RefreshBinding(uid);
    }

    private List<EntityUid> GetSubtree(EntityUid root)
    {
        var result = new List<EntityUid>();
        var pending = new Stack<EntityUid>();
        pending.Push(root);
        while (pending.TryPop(out var uid))
        {
            result.Add(uid);
            using var children = Transform(uid).ChildEnumerator;
            while (children.MoveNext(out var child))
                pending.Push(child);
        }
        return result;
    }

    private void RefreshBinding(EntityUid uid)
    {
        if (TerminatingOrDeleted(uid) || _containers.IsEntityOrParentInContainer(uid))
        {
            RemoveBinding(uid);
            return;
        }
        if (MetaData(uid).EntityLifeStage < EntityLifeStage.Initialized)
            return;
        var hasBody = TryComp<SpatialBodyComponent>(uid, out var body);
        var hasGeometry = HasComp<SpatialGeometryComponent>(uid);
        if (!hasBody && !hasGeometry)
            return;
        if (hasBody && body!.LifeStage < ComponentLifeStage.Initialized)
            return;
        var map = SpatialPhysicsOwnership.GetMap(EntityManager, uid);
        if (map == null)
        {
            RemoveBinding(uid);
            return;
        }
        if (!_worlds.TryGetValue(map.Value, out var backend))
        {
            RemoveBinding(uid);
            if (hasBody)
                throw new InvalidOperationException("A spatial body requires an active spatial physics world.");
            return;
        }

        var definition = hasBody ? body!._definition : new SpatialBodyDefinition(SpatialMotionOwner.Static, useGeometry: true);
        var linear = hasBody ? body!._linearVelocity : Vector3.Zero;
        var angular = hasBody ? body!._angularVelocity : Vector3.Zero;
        ValidateDefinition(uid, definition, linear, angular);
        if (hasGeometry && !definition.UseGeometry)
            throw new InvalidOperationException("A frame with canonical geometry must use that geometry for its body.");
        var paused = hasBody && MetaData(uid).EntityPaused;
        var state = new SpatialBackendState(_transforms.GetWorldPose(uid), paused ? Vector3.Zero : linear,
            paused ? Vector3.Zero : angular);
        if (_bindings.TryGetValue(uid, out var existing) && existing.World == map)
        {
            if (existing.Paused != paused)
            {
                if (definition.Owner == SpatialMotionOwner.DynamicBody)
                    backend.SetPaused(existing.Body, paused);
                _bindings[uid] = existing with { Paused = paused };
            }
            if (backend.GetState(existing.Body) == state)
                return;
            backend.SetState(existing.Body, state);
            return;
        }

        var parts = new List<SpatialBackendPart>();
        SpatialPhysicsOwnership.ValidateReparent(EntityManager, uid, Transform(uid).ParentUid);
        ulong revision = 0;
        if (definition.UseGeometry)
        {
            // A moving structure's geometry belongs to its own compound frame.
            // Nested structural aggregation is handled by the frame/constraint path.
            for (var ancestor = Transform(uid).ParentUid; ancestor.IsValid(); ancestor = Transform(ancestor).ParentUid)
            {
                if (definition.Owner == SpatialMotionOwner.Static && HasComp<SpatialBodyComponent>(ancestor))
                    throw new InvalidOperationException("Structural geometry under another physical body requires compound ownership.");
            }
            var snapshot = _geometry.CaptureSnapshot(uid);
            revision = snapshot.Revision;
            var volume = snapshot.SolidParts.Sum(s => s.Bounds.Volume);
            foreach (var solid in snapshot.SolidParts)
                parts.Add(new SpatialBackendPart(new SpatialBox(solid.Bounds.Size),
                    new Maths.RigidTransform3(solid.Bounds.Center, Quaternion.Identity), solid.Entity ?? uid,
                    solid.Surface, solid.Revision, (float) (solid.Bounds.Volume / volume), solid.MaterialProfile));
        }
        else
        {
            foreach (var collider in definition.Colliders)
                parts.Add(new SpatialBackendPart(collider.Shape.ToBackend(), collider.LocalPose, uid,
                    MassWeight: collider.MassWeight));
        }
        if (parts.Count == 0)
        {
            if (definition.Owner != SpatialMotionOwner.Static)
                throw new InvalidOperationException("A dynamic body requires nonempty solid geometry.");
            RemoveBinding(uid);
            return;
        }
        var backendDefinition = new SpatialBackendBodyDefinition(
            definition.Owner == SpatialMotionOwner.Static ? SpatialBackendMotion.Static : SpatialBackendMotion.Dynamic,
            parts, definition.Mass, definition.Material.ToBackend(), definition.CollisionLayer, definition.CollisionMask);
        var id = backend.AddBody(backendDefinition, state);
        if (paused && definition.Owner == SpatialMotionOwner.DynamicBody)
            backend.SetPaused(id, true);
        RemoveBinding(uid);
        _bindings.Add(uid, new Binding(uid, map.Value, id, definition.UseGeometry, revision, paused));
    }

    private void RemoveBinding(EntityUid uid)
    {
        if (_bindings.Remove(uid, out var binding) && _worlds.TryGetValue(binding.World, out var backend))
            backend.RemoveBody(binding.Body);
    }

    private void ValidateDefinition(EntityUid uid, SpatialBodyDefinition definition, Vector3 linear, Vector3 angular)
    {
        ArgumentNullException.ThrowIfNull(definition);
        definition.Validate();
        if (HasComp<PhysicsComponent>(uid))
            throw new InvalidOperationException("An entity cannot have both planar and spatial physics owners.");
        if (definition.Owner == SpatialMotionOwner.DynamicBody && (Transform(uid).NoLocalRotation || Transform(uid).Anchored))
            throw new InvalidOperationException("Dynamic spatial motion requires physics-owned rotation and compound-aware anchoring.");
        if (definition.Owner is SpatialMotionOwner.Motor or SpatialMotionOwner.AnchoredCompoundChild)
            throw new InvalidOperationException("This movement owner requires its motor or compound-attachment integration.");
        if (!SpatialBackendBodyDefinition.Finite(linear) || !SpatialBackendBodyDefinition.Finite(angular)
            || definition.Owner == SpatialMotionOwner.Static && (linear != Vector3.Zero || angular != Vector3.Zero))
            throw new ArgumentException("Invalid spatial body velocity.");
    }

    internal ISpatialPhysicsBackend GetBackend(EntityUid world) => _worlds[world];
    internal bool TryGetBackend(EntityUid world, [System.Diagnostics.CodeAnalysis.NotNullWhen(true)] out ISpatialPhysicsBackend? backend)
    {
        var found = _worlds.TryGetValue(world, out var value);
        backend = value;
        return found;
    }
    internal int GetBodyCount(EntityUid world) => _worlds[world].BodyCount;

    private void RequireServer()
    {
        if (!_net.IsServer)
            throw new InvalidOperationException("Authoritative spatial physics changes require the server.");
    }

    private void OnWorldGetState(EntityUid uid, SpatialPhysicsWorldComponent component, ref ComponentGetState args)
        => args.State = new SpatialPhysicsWorldComponentState(component._settings);

    private void OnWorldHandleState(EntityUid uid, SpatialPhysicsWorldComponent component, ref ComponentHandleState args)
    {
        if (args.Current is not SpatialPhysicsWorldComponentState state)
            return;
        state.Settings.Validate();
        if (_net.IsServer)
            throw new InvalidOperationException("Server physics settings are not overwritten by component state application.");
        component._settings = state.Settings;
    }

    private void OnBodyGetState(EntityUid uid, SpatialBodyComponent component, ref ComponentGetState args)
        => args.State = new SpatialBodyComponentState(component._definition.Copy(), component._linearVelocity, component._angularVelocity);

    private void OnBodyHandleState(EntityUid uid, SpatialBodyComponent component, ref ComponentHandleState args)
    {
        if (args.Current is not SpatialBodyComponentState state)
            return;
        ValidateDefinition(uid, state.Definition, state.LinearVelocity, state.AngularVelocity);
        if (_net.IsServer)
            throw new InvalidOperationException("Server body state is not overwritten by component state application.");
        var copy = state.Definition.Copy();
        component._definition = copy;
        component._linearVelocity = state.LinearVelocity;
        component._angularVelocity = state.AngularVelocity;
    }
}
