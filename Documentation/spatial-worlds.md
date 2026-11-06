# Spatial world identity and layer context (E01, in progress)

`SpatialWorldComponent` belongs to the existing map entity. Its persistent UUID
is distinct from runtime MapId/EntityUid and wire NetEntity. `SharedSpatialSystem`
creates authoritative worlds on the server, resolves world membership through
Transform.MapUid, and maintains a derived identity lookup that is removed on
shutdown. No entity positions or physics handles are stored in this lookup.

`SpatialFrameComponent` supplies layer definitions on an existing transform frame.
Each local `LayerId` identifies an immutable `LayerDefinition`: local elevation,
display name and horizontal authoring-grid size in metres. Elevations need not be
uniform, positive or unique. There is no division by a constant floor height.
Runtime layer references combine the owner EntityUid with the local LayerId;
network references use NetEntity. Map/prefab serialization remaps the owner.

`GetLayerWorldPose` and `GetPoseRelativeToLayer` compute a layer's frame and relative
pose without changing any entity. `SetLayerPlanarPosition` preserves signed depth
in that selected plane and world orientation, including tilted frames. Cross-world
conversion is rejected. Choosing a layer does not select physical support or
teleport an entity. Transferring a frame uses the existing transform hierarchy;
layer references follow its new world membership.

Initial layer definitions can be configured only before entity initialization.
The public collection is read-only and state application validates a copied
dictionary before replacing data. Runtime structural edits remain the responsibility
of the future geometry transaction service; there is no public live-layer setter
that bypasses that commit path.

## Persistence and networking

`definitionSchema: 1` versions this metadata. It is not the complete geometry codec
or collision revision. Normal loading restores the saved persistent world UUID.
Loading a second active world with that ID fails and cleans up newly allocated
entities. `DeserializationOptions.RegenerateSpatialWorldIds` explicitly clones
worlds with new persistent IDs while preserving all intra-copy frame/layer links.
Ordinary non-map prefabs continue to use the existing entity remapping path.

World and frame component states replicate persistent identity and layer definitions.
The frame is identified by the normal component owner. Invalid IDs, schemas and
layer definitions are rejected before replacing existing metadata. The client cannot
create an authoritative world through `CreateWorld`; it receives the server's state.

The full Shared integration suite passed with these additions: 1242 tests. Five
targeted cases also pass after the final lifecycle guards, including a connected
client, network deletion, save/restore/clone, collision of persistent IDs, immutable
definitions, cross-world rejection, tilted layer projection and continuous poses.

## Remaining integration

These components describe spatial identity and authoring context. They do not yet
activate a completed 3D physics/query/render profile. Implicit planar-call diagnostics,
surface identities and geometry, transaction revisions and ownership of XYZ physics
velocities remain unfinished. Neither E01/G1 nor E02/G2 is closed by this metadata.
