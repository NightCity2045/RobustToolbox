# Spatial interest and the existing PVS pipeline

`SpatialInterestSystem` implements the server-side `ISpatialInterestService`.
Its per-world entity and semantic-surface volume trees contain derived bounds,
not authoritative poses. Transform remains the only pose source. Geometry is
captured by revision and component identity; surface-local bounds are reused
when a frame moves. The physics solver's broadphase is not used as the entity
interest index: nonphysical entities, paused bodies and authoring geometry also
need replication.

Initialization, movement, rotation, parent/world transfer, body reconfiguration,
geometry transaction completion and component/entity removal invalidate the
affected entries. Parent changes include descendants. Ordinary map entity
changes do not populate the spatial dirty queue. Main-thread preparation finishes
before parallel PVS reads. Queries have independent traversal scratch storage
and no fixed unsafe stack-depth limit; BEPU handles and pooled memory stay private.

For maps with `SpatialWorld`, normal PVS chunk selection is replaced by XYZ
volume queries. The existing attached entity and server-authorized
`ViewSubscriptions` supply the viewers. Interest includes the surrounding volume,
not just the current camera frustum. The nearest physical ancestor supplies the
viewer's world point velocity once, including angular velocity. Directional
preloading is limited by `net.spatial_preload_seconds` (default 0.5 seconds) and
`net.spatial_preload_distance` (default 64 metres). View ranges retain the existing
server PVS settings. These bounds do not authorize a viewer or bypass visibility.

Normal spatial candidates must satisfy visibility masks and ancestor visibility.
Container descendants default to denied. Content can provide a read-only
`SpatialInterestVisibilityEvent` handler to authorize a specific session/entity
or further restrict gameplay visibility. This callback runs during parallel PVS
generation and must not mutate entities. Existing explicit content PVS overrides
and forced viewers retain their semantics. The server's existing PVS-off/debug
path remains an explicit unculled replication mode; it is not a client quality
setting or the normal spatial interest route.

Parents and motor support entities/frames are added before dependent entities
under the existing entry budgets. Entity states still use the existing net IDs,
acknowledgements, delta/full-state generation and leave/re-entry machinery. No
second network entity representation is introduced.

`Collect` is a main-thread candidate query that prepares current derived indexes
and requires an explicit entity-policy predicate. Its surface results identify
semantic geometry needed in the selected volume. They do not yet constitute
the geometry chunk transport, collision-readiness barrier, reconnect revision
baseline, prediction/reconciliation or historical-query implementation. Those
remaining E07 requirements must be completed before G7 acceptance.

Coverage includes randomized insert/move/remove against brute force, concurrent
readers, real client/server height culling, parent movement and re-entry, hidden
layers and container contents, world transfers, collider reconfiguration,
surface bounds away from frame origins, geometry commits/removal, authorized
remote camera subscription/revocation and bounded vehicle-camera preloading.
