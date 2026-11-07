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
/// The client motor driver reuses the backend under the existing reset/replay boundary.
/// </summary>
public sealed partial class SharedSpatialPhysicsSystem : EntitySystem
{
    [Dependency] private INetManager _net = default!;
    [Dependency] private SharedTransformSystem _transforms = default!;
    [Dependency] private SharedGeometrySystem _geometry = default!;
    [Dependency] private SharedContainerSystem _containers = default!;

    private readonly Dictionary<EntityUid, BepuSpatialPhysicsBackend> _worlds = new();
    private readonly Dictionary<EntityUid, Binding> _bindings = new();
    private readonly Dictionary<EntityUid, EntityUid> _compoundOwners = new();
    private bool _committing;
    private bool _updating;

    private sealed record Binding(EntityUid Entity, EntityUid World, SpatialBackendBodyId Body,
        bool Geometry, ulong GeometryRevision, bool Paused, ulong DefinitionRevision, CompoundCapture? Compound = null);

    public override void Initialize()
    {
        base.Initialize();
        InitializeConstraints();
        InitializeMotors();
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
        SubscribeLocalEvent<SpatialGeometryComponent, SpatialGeometryStopping>(OnGeometryShutdown);
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
        _compoundOwners.Clear();
        _constraintBindings.Clear();
        _constraintComponents.Clear();
        _semanticContacts.Clear();
        _motors.Clear();
        _motorIncomingVelocity.Clear();
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
        RequireMotorAuthority(body);
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
        if (!float.IsFinite(frameTime))
            throw new ArgumentOutOfRangeException(nameof(frameTime));
        if (_updating || _transferring)
            throw new InvalidOperationException("The spatial physics boundary cannot be reentered.");
        _updating = true;
        try { UpdateSpatial(frameTime); }
        finally { _updating = false; }
    }

    private void UpdateSpatial(float frameTime)
    {
        _geometry.CommitPending();
        SyncConstraints();
        PrepareMotors(frameTime);
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
                || !IsDynamicOwner(component.Definition.Owner))
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
        PublishMotorSupports();
        PublishContacts();
        var completed = new SpatialPhysicsStepCompleted();
        RaiseLocalEvent(ref completed);
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
            RemoveBinding(binding.Entity);
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
        _compoundOwners.TryGetValue(uid, out var root);
        RemoveConstraintsReferencing(uid);
        RemoveBinding(uid);
        if (root.IsValid() && !TerminatingOrDeleted(root)) RefreshBinding(root);
    }

    private void OnGeometryStartup(EntityUid uid, SpatialGeometryComponent component, ComponentStartup args)
    {
        if (_net.IsServer && (!TryComp<SpatialBodyComponent>(uid, out var body)
            || body.HasDefinition && body.Definition.UseGeometry))
            RefreshBinding(uid);
    }

    private void OnGeometryShutdown(EntityUid uid, SpatialGeometryComponent component, ref SpatialGeometryStopping args)
    {
        _compoundOwners.TryGetValue(uid, out var root);
        RemoveConstraintsReferencing(uid);
        if (_bindings.TryGetValue(uid, out var binding) && binding.Geometry)
            RemoveBinding(uid);
        if (root.IsValid() && !TerminatingOrDeleted(root)) RefreshBinding(root);
    }

    private void OnMove(ref MoveEvent args)
    {
        if (!_net.IsServer || _committing)
            return;
        var root = args.Sender;
        // If the owner moved, independent bodies keep their solver world pose.
        // Contained bodies are dormant and continue to follow container transforms.
        var subtree = GetSubtree(root);
        var oldRoots = CompoundOwnersOf(subtree);
        _committing = true;
        try
        {
            foreach (var uid in subtree)
            {
                if (uid == root || !_bindings.TryGetValue(uid, out var binding) || TerminatingOrDeleted(uid)
                    || !TryComp<SpatialBodyComponent>(uid, out var body) || !IsDynamicOwner(body.Definition.Owner)
                    || _containers.IsEntityOrParentInContainer(uid) || !_worlds.TryGetValue(binding.World, out var backend))
                    continue;
                _transforms.SetWorldPose(binding.Entity, backend.GetState(binding.Body).Pose);
            }
        }
        finally { _committing = false; }
        foreach (var uid in subtree)
            if (EntityManager.EntityExists(uid))
                RefreshBinding(uid);
        foreach (var previousRoot in oldRoots)
            if (!TerminatingOrDeleted(previousRoot)) RefreshBinding(previousRoot);
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
        var subtree = GetSubtree(root);
        var oldRoots = CompoundOwnersOf(subtree);
        foreach (var uid in subtree)
            RefreshBinding(uid);
        foreach (var previousRoot in oldRoots)
            if (!TerminatingOrDeleted(previousRoot)) RefreshBinding(previousRoot);
    }

    private HashSet<EntityUid> CompoundOwnersOf(IEnumerable<EntityUid> entities)
    {
        var owners = new HashSet<EntityUid>();
        foreach (var uid in entities)
            if (_compoundOwners.TryGetValue(uid, out var root)) owners.Add(root);
        return owners;
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
        if (hasBody && body!.Definition.Owner == SpatialMotionOwner.AnchoredCompoundChild)
        {
            RemoveBinding(uid);
            if (FindCompoundRoot(uid) is { } root) RefreshBinding(root);
            return;
        }
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
        ValidateGeometryOwnership(uid, definition);
        var definitionRevision = hasBody ? body!._definitionRevision : 0;
        var paused = hasBody && MetaData(uid).EntityPaused;
        var state = new SpatialBackendState(_transforms.GetWorldPose(uid), paused ? Vector3.Zero : linear,
            paused ? Vector3.Zero : angular);
        if (_bindings.TryGetValue(uid, out var existing) && existing.World == map && existing.DefinitionRevision == definitionRevision
            && (!hasGeometry || existing.GeometryRevision == Comp<SpatialGeometryComponent>(uid).Revision)
            && (existing.Compound == null || CompoundStillCurrent(existing.Compound)))
        {
            if (existing.Paused != paused)
            {
                if (IsDynamicOwner(definition.Owner))
                    backend.SetPaused(existing.Body, paused);
                _bindings[uid] = existing with { Paused = paused };
            }
            if (backend.GetState(existing.Body) == state)
                return;
            backend.SetState(existing.Body, state);
            return;
        }

        var compound = CaptureCompound(uid, definition, definitionRevision);
        var backendDefinition = compound.Definition;
        var revision = hasGeometry ? Comp<SpatialGeometryComponent>(uid).Revision : 0;
        if (backendDefinition == null)
        {
            RemoveBinding(uid);
            return;
        }
        var id = backend.AddBody(backendDefinition, state);
        if (paused && IsDynamicOwner(definition.Owner))
            backend.SetPaused(id, true);
        RemoveBinding(uid);
        InstallBinding(new Binding(uid, map.Value, id, definition.UseGeometry, revision, paused, definitionRevision, compound));
    }

    private void ValidateGeometryOwnership(EntityUid uid, SpatialBodyDefinition definition)
    {
        if (HasComp<SpatialGeometryComponent>(uid) && !definition.UseGeometry)
            throw new InvalidOperationException("A frame with canonical geometry must use that geometry for its body.");
        SpatialPhysicsOwnership.ValidateReparent(EntityManager, uid, Transform(uid).ParentUid);
        if (definition.Owner == SpatialMotionOwner.Static)
        {
            for (var ancestor = Transform(uid).ParentUid; ancestor.IsValid(); ancestor = Transform(ancestor).ParentUid)
                if (HasComp<SpatialBodyComponent>(ancestor))
                    throw new InvalidOperationException("Structural geometry under another physical body requires compound ownership.");
        }
    }

    private SpatialBackendBodyDefinition? BuildBackendDefinition(EntityUid uid, SpatialBodyDefinition definition,
        ulong definitionRevision, out ulong revision, SpatialGeometrySnapshot? geometry = null)
    {
        var parts = new List<SpatialBackendPart>();
        revision = 0;
        if (definition.UseGeometry)
        {
            var snapshot = geometry ?? _geometry.CaptureSnapshot(uid);
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
                    Revision: definitionRevision, MassWeight: collider.MassWeight));
        }
        if (parts.Count == 0)
        {
            if (definition.Owner != SpatialMotionOwner.Static)
                throw new InvalidOperationException("A dynamic body requires nonempty solid geometry.");
            return null;
        }
        return new SpatialBackendBodyDefinition(
            definition.Owner == SpatialMotionOwner.Static ? SpatialBackendMotion.Static : SpatialBackendMotion.Dynamic,
            parts, definition.Mass, definition.Material.ToBackend(), definition.CollisionLayer, definition.CollisionMask,
            lockRotation: definition.Owner == SpatialMotionOwner.Motor);
    }

    private void RemoveBinding(EntityUid uid)
    {
        if (!_bindings.Remove(uid, out var binding)) return;
        if (binding.Compound != null)
            foreach (var member in binding.Compound.Members)
                if (_compoundOwners.TryGetValue(member.Entity, out var root) && root == uid) _compoundOwners.Remove(member.Entity);
        if (_worlds.TryGetValue(binding.World, out var backend)) backend.RemoveBody(binding.Body);
    }

    private void InstallBinding(Binding binding)
    {
        RemoveBinding(binding.Entity);
        if (binding.Compound is { } compound)
        {
            binding = binding with { Compound = WithCommittedCompoundVersions(compound) };
            foreach (var member in compound.Members)
                if (member.Entity != binding.Entity) _compoundOwners[member.Entity] = binding.Entity;
        }
        _bindings.Add(binding.Entity, binding);
    }

    private void ValidateDefinition(EntityUid uid, SpatialBodyDefinition definition, Vector3 linear, Vector3 angular)
    {
        ArgumentNullException.ThrowIfNull(definition);
        definition.Validate();
        if (HasComp<PhysicsComponent>(uid))
            throw new InvalidOperationException("An entity cannot have both planar and spatial physics owners.");
        if (IsDynamicOwner(definition.Owner) && (Transform(uid).NoLocalRotation || Transform(uid).Anchored))
            throw new InvalidOperationException("Dynamic spatial motion requires physics-owned rotation and compound-aware anchoring.");
        if (definition.Owner == SpatialMotionOwner.AnchoredCompoundChild)
        {
            if (linear != Vector3.Zero || angular != Vector3.Zero)
                throw new InvalidOperationException("A compound child derives velocity from its root and cannot own independent motion.");
            if (_net.IsServer)
            {
                var root = FindCompoundRoot(uid) ?? throw new InvalidOperationException("A compound child requires a physical ancestor.");
                if (TryComp<SpatialBodyComponent>(root, out var rootBody) && rootBody.Definition.Owner == SpatialMotionOwner.Motor)
                    throw new InvalidOperationException("A capsule motor cannot own structural compound children.");
            }
        }
        if (definition.Owner == SpatialMotionOwner.Motor)
            ValidateMotorBody(uid, definition, angular);
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
    internal bool HasActiveCollision(EntityUid entity) => _bindings.ContainsKey(entity) || _compoundOwners.ContainsKey(entity);

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
        => args.State = new SpatialBodyComponentState(component._definition.Copy(), component._linearVelocity, component._angularVelocity,
            component._definitionRevision);

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
        component._definitionRevision = state.DefinitionRevision;
    }
}
