# Spatial geometry addressing (E02, in progress)

`BoundaryKey` implements the canonical address from architecture §6.2:
owner frame, local layer ID, integer X/Y and Horizontal/Vertical axis. X/Y
identify the lesser endpoint of a unit edge in the authoring grid. Horizontal
extends toward +X and Vertical toward +Y. Adjacent cells share one key for their
common edge; reversing its endpoints does not produce another construction slot.

`LayerId` is a stable unsigned local identifier. It is not an elevation, map ID,
or globally unique identifier. A layer address includes its owner frame. The
owner uses the existing EntityUid runtime reference; the entity serializer writes
file-local IDs and remaps them independently for each prefab instance. A boundary
key is deliberately not marked NetSerializable: a geometry wire descriptor must
use NetEntity for the owner instead of transmitting runtime EntityUid values.

Prefab grid transforms mirror X, apply a counterclockwise quarter-turn rotation,
then translate and normalize endpoints. Arithmetic rejects unrepresentable grid
coordinates instead of wrapping into another construction slot. Wider intermediate
values permit transformations whose final endpoints fit despite an intermediate
reflection exceeding Int32. Arbitrary world rotation remains in the owner frame's
canonical transform and does not rewrite its local boundary addresses.

Eight unit cases cover shared edges, negative coordinates, all grid symmetries,
known rotation/reflection results, frame/layer scope, T-junction edge identity,
invalid edges, overflow, and stable scalar layer IDs. An integration fixture saves
a real entity prefab and loads two independent copies, checking owner remapping,
local layer IDs, unchanged endpoints, and rejection/cleanup of an invalid saved
edge. All 533 Shared unit tests and 1237 Shared integration tests pass.
This is address normalization, not
generation of corner meshes or removal of internal mesh faces.

E02/G2 is not complete. Layer definitions and sparse storage, surface identities,
segments with thickness and real openings, slabs, revisioned geometry snapshots,
and their physics/query/render consumers remain to be implemented. The address
types do not create a second world or a second source of entity coordinates.
