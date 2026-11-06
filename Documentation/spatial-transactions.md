# Structural geometry transactions

`SharedGeometrySystem.PrepareTransaction` accepts one or more
`SpatialGeometryEdit` replacements built from live captures. All frames must belong
to the same spatial world. Preparation copies and validates geometry, references,
layers and coverings, assigns authoritative surface/chunk revisions, and prepares
physical shapes outside broadphase. It neither changes canonical collections nor
inserts a temporary body. Abandoned transactions must be disposed.

`QueueTransaction` requests installation at the next pre-physics boundary. Keep
the transaction alive until completion: disposing a queued transaction cancels it.
The service rechecks expected revisions, component identities, world membership,
source entities and physical definitions before installing any frame. A stale
member rejects the entire transaction. A change of world physics profile also
invalidates preparation, including enabling physics after an authoring-only edit
was prepared. Cancellation and frame/world deletion release prepared resources.

Replacement bodies are validated against current canonical poses and velocities,
not their earlier values at preparation time. After all preparation succeeds,
canonical geometry/layers and colliders are installed without gameplay callbacks
between frames. Constraint synchronization, motor support queries and physical
movement then see the new geometry in that same tick. The server never waits for
a mesh worker. Decorative tile-only changes retain existing collider identity.

`WorldEditCompleted` reports Committed, Rejected or Cancelled queued work. It runs
after the batch is installed; public queries already see the matching revisions
and colliders. Completion handlers may queue another edit for the following
boundary. Recursive physics/structural boundaries are rejected. Content retains
responsibility for access, resources, occupancy policy and coordination of its
own gameplay changes; no inventory or health operations occur here.

Each `SpatialGeometryChange` provides immutable before/after captures, changed
surface IDs, authoring layers, decorative chunk keys and old/new local solid
bounds. Consumers use these to invalidate their specialized caches immediately;
an expensive rebuilt result must pass `IsSnapshotCurrent` before installation.
The guard includes an opaque component capture identity, so removing/readding a
geometry component with the same numerical revision cannot revive an old result.
Empty/open surfaces remain represented by changed surface IDs even without bounds.

`CreateInverse` returns structural replacements guarded by the committed revision.
Applying them creates a new revision and restamps changed surfaces/chunks. It does
not restore earlier body motion, recreate deleted source entities or roll back
gameplay state. Newer edits or unavailable sources reject the inverse normally.

These contracts implement the technical E04 boundary. Geometry streaming and its
revision acknowledgements belong to E07; renderer/cache consumers and the content
gameplay coordinator still need their respective integrations before G9 closes.
