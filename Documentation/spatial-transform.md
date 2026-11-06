# Canonical pose migration (E01, in progress)

The existing `TransformComponent` now owns a `Vector3` position and a normalized
`Quaternion` orientation relative to its existing parent. There is no additional
height component or authoritative position table. The old XY/yaw properties are
derived views; their writes preserve height and tilt. These legacy entry points
still need the explicit plane/world diagnostics required by the spatial plan.

`SharedTransformSystem.GetWorldPose`, `SetLocalPose`, `SetWorldPose`,
`SetParentLocalPose`, and `ConvertPoint` operate on the full pose. `SetParent`
uses the full hierarchy for world-preserving reparenting. `GetPoseRelativeTo`,
`GetPlanarPosition`, and `SetPlanarPosition` explicitly select a frame in the same
world hierarchy. A planar write preserves signed depth in that frame and world
orientation, including when the selected plane is tilted. Cross-world projection
is rejected before a write. `GetWorldMatrix3`/`GetInvWorldMatrix3` are derived
matrices, not additional state. Existing anchoring and
`NoLocalRotation` restrictions still apply. `MoveEvent` carries both full local
poses alongside its legacy projections; a height-only move is not a rotation-only
event. Layer selection is not involved in these transformations.

Full and delta network states contain XYZ/XYZW, using the existing dirty-field
indices. Prediction snapshots and interpolation contain the same types;
interpolation uses `Vector3.Lerp` and `Quaternion.Slerp`. A client built against
the old transform state must not be paired with this engine. The existing engine
serializer type hash changes with the state layout.

## Persistence

This fork writes entity/map format **8**, with metadata `transformSchema: 1`.
Format 8 without that supported schema is rejected before allocating entities.
This is the transform schema, not the future geometry/map codec from E09.
Tile chunk encoding remains at its existing version.

The transform fields retain the YAML names `pos` and `rot`:

- `pos`: canonical `x,y,z`; legacy `x,y` imports with `z=0`.
- `rot`: canonical normalized `x,y,z,w`; legacy degrees and `rad` angles import
  as yaw about +Z. Zero/non-finite quaternions and non-finite coordinates fail
  validation and reading. Saving invalid state fails instead of changing it.

Existing YAML entity IDs still remap parent references on every load. Runtime
EntityUid and network NetEntity remain separate. Maps using formats 3–7 continue
through the existing compatibility readers. Quaternion float precision means
angle equality is rotational equality, not preservation of an arbitrary signed
angle representation. Flat polygon contact faces can have multiple valid contact
witnesses; tests check the contact segment and a one-micrometre position tolerance.

## Verification and remaining work

The complete local engine suite passes: **5160 passed, 0 failed**; one existing
explicit client test is not executed. New tests cover nested frames, reparenting,
height-preserving planar writes, full/delta corrections, a no-rotation transition,
network byte round trips, two independent loads of an XYZ map, invalid mutations,
and a connected client's height/pitch interpolation. Legacy map tests also pass.
Two subsequent tests verify depth-preserving writes on a tilted plane and atomic
cross-world rejection; all six spatial transform tests pass.

E01/G1 is not complete. Spatial-world profiles, stable world/layer/surface IDs,
diagnostics for implicit planar calls, and full movement ownership/lifecycle
integration remain. The existing physics, query, renderer, and PVS implementations
are still planar; this pose migration does not qualify them as spatial systems.
Frame velocity mathematics is implemented, but physics-owned XYZ velocities are
not yet integrated into ECS. Full gameplay and performance gates remain open.
