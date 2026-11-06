using System;
using System.Collections.Generic;
using System.Numerics;
using Robust.Shared.GameObjects;
using Robust.Shared.GameStates;
using Robust.Shared.IoC;
using Robust.Shared.Map.Components;
using Robust.Shared.Maths;
using Robust.Shared.Network;

namespace Robust.Shared.Spatial;

/// <summary>World identity and authoring-layer context on the existing entity hierarchy.</summary>
public sealed partial class SharedSpatialSystem : EntitySystem
{
    [Dependency] private SharedMapSystem _maps = default!;
    [Dependency] private SharedTransformSystem _transforms = default!;
    [Dependency] private INetManager _net = default!;

    // Derived lookup, not an additional authoritative world or transform table.
    private readonly Dictionary<PersistentWorldId, EntityUid> _worlds = new();

    public override void Initialize()
    {
        base.Initialize();
        SubscribeLocalEvent<SpatialWorldComponent, ComponentInit>(OnWorldInit);
        SubscribeLocalEvent<SpatialWorldComponent, ComponentShutdown>(OnWorldShutdown);
        SubscribeLocalEvent<SpatialWorldComponent, ComponentGetState>(OnWorldGetState);
        SubscribeLocalEvent<SpatialWorldComponent, ComponentHandleState>(OnWorldHandleState);
        SubscribeLocalEvent<SpatialFrameComponent, ComponentInit>(OnFrameInit);
        SubscribeLocalEvent<SpatialFrameComponent, ComponentGetState>(OnFrameGetState);
        SubscribeLocalEvent<SpatialFrameComponent, ComponentHandleState>(OnFrameHandleState);
    }

    public override void Shutdown()
    {
        _worlds.Clear();
        base.Shutdown();
    }

    public Entity<SpatialWorldComponent> CreateWorld()
    {
        if (!_net.IsServer)
            throw new InvalidOperationException("Authoritative spatial worlds are created by the server.");
        var uid = _maps.CreateMap();
        try
        {
            return (uid, EnsureComp<SpatialWorldComponent>(uid));
        }
        catch
        {
            Del(uid);
            throw;
        }
    }

    /// <summary>Resolve current world membership from Transform, including after a frame transfer.</summary>
    public Entity<SpatialWorldComponent> GetWorld(EntityUid uid)
    {
        if (TerminatingOrDeleted(uid))
            throw new InvalidOperationException("A terminating entity has no usable spatial world context.");
        var map = Transform(uid).MapUid;
        if (map == null || TerminatingOrDeleted(map.Value) || !TryComp<SpatialWorldComponent>(map.Value, out var world))
            throw new InvalidOperationException("Entity is not in a spatial world.");
        return (map.Value, world);
    }

    public bool TryResolveWorld(PersistentWorldId id, out EntityUid uid)
    {
        if (_worlds.TryGetValue(id, out uid) && !TerminatingOrDeleted(uid))
            return true;
        uid = EntityUid.Invalid;
        return false;
    }

    /// <summary>
    /// Configure authoring definitions before initialization. Live structural edits
    /// must later use the geometry transaction service, not bypass it through this API.
    /// </summary>
    public void ConfigureInitialLayers(Entity<SpatialFrameComponent> frame,
        IEnumerable<KeyValuePair<LayerId, LayerDefinition>> definitions)
    {
        if (MetaData(frame).EntityLifeStage != EntityLifeStage.PreInit)
            throw new InvalidOperationException("Initial layers must be configured before entity initialization.");
        var layers = ValidateLayers(definitions);
        frame.Comp._layers = layers;
        Dirty(frame);
    }

    public LayerDefinition GetLayer(SpatialLayerReference reference)
    {
        GetWorld(reference.OwnerFrame);
        return Comp<SpatialFrameComponent>(reference.OwnerFrame)._layers[reference.Layer];
    }

    public RigidTransform3 GetLayerWorldPose(SpatialLayerReference reference)
    {
        var layer = GetLayer(reference);
        return _transforms.GetWorldPose(reference.OwnerFrame).Compose(
            new RigidTransform3(new Vector3(0, 0, layer.LocalElevation), Quaternion.Identity));
    }

    public RigidTransform3 GetPoseRelativeToLayer(EntityUid uid, SpatialLayerReference reference)
    {
        RequireSameWorld(uid, reference.OwnerFrame);
        return GetLayerWorldPose(reference).ToLocal(_transforms.GetWorldPose(uid));
    }

    /// <summary>Explicit planar write preserving depth in this layer and full world orientation.</summary>
    public void SetLayerPlanarPosition(EntityUid uid, SpatialLayerReference reference, Vector2 position)
    {
        SharedTransformSystem.ValidatePosition(position);
        var relative = GetPoseRelativeToLayer(uid, reference);
        var updated = new RigidTransform3(new Vector3(position, relative.Position.Z), relative.Rotation);
        _transforms.SetWorldPose(uid, GetLayerWorldPose(reference).Compose(updated));
    }

    public NetSpatialLayerReference ToNetwork(SpatialLayerReference reference)
    {
        GetLayer(reference);
        return new NetSpatialLayerReference(GetNetEntity(reference.OwnerFrame), reference.Layer);
    }

    public SpatialLayerReference FromNetwork(NetSpatialLayerReference reference)
    {
        var resolved = new SpatialLayerReference(GetEntity(reference.OwnerFrame), reference.Layer);
        GetLayer(resolved);
        return resolved;
    }

    private void RequireSameWorld(EntityUid first, EntityUid second)
    {
        if (GetWorld(first).Owner != GetWorld(second).Owner)
            throw new InvalidOperationException("Layer conversion requires entities in the same spatial world.");
    }

    private static Dictionary<LayerId, LayerDefinition> ValidateLayers(
        IEnumerable<KeyValuePair<LayerId, LayerDefinition>> definitions)
    {
        var result = new Dictionary<LayerId, LayerDefinition>();
        foreach (var (id, definition) in definitions)
        {
            definition.Validate();
            result.Add(id, definition);
        }
        return result;
    }

    internal void PrepareLoadedWorlds(IEnumerable<EntityUid> entities, bool regenerateIds)
    {
        var assigned = new HashSet<PersistentWorldId>();
        foreach (var uid in entities)
        {
            if (!TryComp<SpatialWorldComponent>(uid, out var component))
                continue;
            ValidateWorldDefinition(uid, component._definitionSchema);
            var id = component._persistentId;
            if (regenerateIds || !id.IsValid)
                id = PersistentWorldId.New();
            if (_worlds.ContainsKey(id) || !assigned.Add(id))
                throw new InvalidOperationException("Duplicate persistent spatial world ID. Clone with RegenerateSpatialWorldIds enabled.");
            component._persistentId = id;
        }
    }

    private void ValidateWorldDefinition(EntityUid uid, uint schema)
    {
        if (!HasComp<MapComponent>(uid))
            throw new InvalidOperationException("SpatialWorld must belong to the existing map entity.");
        if (schema != SpatialWorldComponent.CurrentDefinitionSchema)
            throw new InvalidOperationException($"Unsupported spatial definition schema {schema}.");
    }

    private void RegisterWorld(EntityUid uid, SpatialWorldComponent component, PersistentWorldId id, uint schema)
    {
        ValidateWorldDefinition(uid, schema);
        if (!id.IsValid)
            throw new InvalidOperationException("A spatial world requires a persistent ID.");
        if (_worlds.TryGetValue(id, out var existing) && existing != uid)
            throw new InvalidOperationException("Duplicate persistent spatial world ID.");
        UnregisterWorld(uid, component._persistentId);
        component._persistentId = id;
        component._definitionSchema = schema;
        _worlds[id] = uid;
    }

    private void UnregisterWorld(EntityUid uid, PersistentWorldId id)
    {
        if (_worlds.TryGetValue(id, out var registered) && registered == uid)
            _worlds.Remove(id);
    }

    private void OnWorldInit(EntityUid uid, SpatialWorldComponent component, ComponentInit args)
    {
        ValidateWorldDefinition(uid, component._definitionSchema);
        if (!_net.IsServer && !component._persistentId.IsValid)
            return; // The authoritative state assigns the ID on network clients.
        RegisterWorld(uid, component,
            component._persistentId.IsValid ? component._persistentId : PersistentWorldId.New(), component._definitionSchema);
        Dirty(uid, component);
    }

    private void OnWorldShutdown(EntityUid uid, SpatialWorldComponent component, ComponentShutdown args)
    {
        var ev = new SpatialWorldStopping();
        RaiseLocalEvent(uid, ref ev);
        UnregisterWorld(uid, component._persistentId);
    }

    private void OnWorldGetState(EntityUid uid, SpatialWorldComponent component, ref ComponentGetState args)
        => args.State = new SpatialWorldComponentState(component._persistentId, component._definitionSchema);

    internal void OnWorldHandleState(EntityUid uid, SpatialWorldComponent component, ref ComponentHandleState args)
    {
        if (args.Current is SpatialWorldComponentState state)
            RegisterWorld(uid, component, state.PersistentId, state.DefinitionSchema);
    }

    private void OnFrameInit(EntityUid uid, SpatialFrameComponent component, ComponentInit args)
        => component._layers = ValidateLayers(component._layers);

    private void OnFrameGetState(EntityUid uid, SpatialFrameComponent component, ref ComponentGetState args)
    {
        var stamp = TryComp<SpatialGeometryComponent>(uid, out var geometry) && geometry.Running
            ? new SpatialGeometryStamp(geometry.WireGeneration, geometry.Revision) : (SpatialGeometryStamp?) null;
        args.State = new SpatialFrameComponentState(component._layers, stamp);
    }

    internal void OnFrameHandleState(EntityUid uid, SpatialFrameComponent component, ref ComponentHandleState args)
    {
        if (args.Current is not SpatialFrameComponentState state) return;
        var layers = ValidateLayers(state.Layers);
        if (state.Geometry is { Generation: var generation } && generation == Guid.Empty)
            throw new InvalidOperationException("Geometry state requires a nonempty wire generation.");
        component._layers = layers;
        if (component.ReceivedGeometryStamp == state.Geometry) return;
        component.ReceivedGeometryStamp = state.Geometry;
        var ev = new SpatialGeometryStampChanged(state.Geometry);
        RaiseLocalEvent(uid, ref ev);
    }
}
