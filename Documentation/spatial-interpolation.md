# Authoritative pose presentation

`SpatialPoseTimeline` retains bounded, coherent batches of full XYZ/quaternion
transforms for one existing spatial world. It is derived presentation data, not a
second transform authority or a prediction backend. Input batches include their
world root and all parent dependencies; missing parents, cycles, invalid poses,
duplicates and mixed worlds are rejected. Limits cover frame count, entities per
frame and total retained samples. Exceeding a sample budget clears old output.

Sampling interpolates position and quaternion rotation between authoritative
ticks. Hierarchical objects compose interpolated local poses, preserving the arc
of an object fixed to a turning parent. Independent bodies interpolate resolved
world poses instead: carrier motion must not be applied to them a second time.
A hierarchy/ownership discontinuity snaps the entire dependent pose to the newer
coherent sample. Exact tick queries retain the exact sample at that tick.

An entity must occur in the newest batch to be sampled at all. New entities are
not fabricated in older batches; times before retained history fail, while times
after the newest sample hold that sample without extrapolation. Invalidation
removes an entity and its parent-dependent descendants from every retained batch.
Clearing is required before installing a new session or a reset tick sequence.

## Client integration

`SpatialPoseHistorySystem.Track(world, selection, limits)` creates a disposable
history for an explicit content-owned view selection. Parent dependencies count
against the same entity budget and cannot be sampled as additional visuals unless
selected. This operation changes no PVS subscriptions and grants no visibility.

The system captures only after `IClientGameStateManager.GameStateApplied`, using
the applied state's `ToSequence`, before local prediction. It reads canonical
`TransformComponent.LocalPose`; it never captures a partially applied batch or
the legacy sprite interpolation offset. Only initialized, non-detached entities
in the selected world are included; client-created entities are excluded. A missing ancestor suppresses its dependents
until a later complete state. Dynamic bodies and motors use world interpolation;
anchored/static members and container-contained descendants use the hierarchy.

`SetSelection` revokes old output immediately when the selection changes. The
next complete state establishes new history. World transfer and deletion
invalidate dependent samples; world deletion, entity flush, system shutdown and
disconnect dispose the history and release its samples. Reconnection requires a
new content-owned selection. A non-increasing applied tick resets retained
history, including replay seeks. Operations belong to the owning client thread.

Content selects the rendering tick/delay and must separately check geometry
streaming readiness and gameplay view permissions. A sampled pose does not
certify that collision chunks are loaded. Local predicted rendering,
reconciliation, input replay and gameplay consequences remain separate work.

Tests cover parent arcs, independent passenger motion, exact ticks, ownership
changes, new entities, parent revocation, bounded eviction, malformed hierarchies
and reset. `SpatialPoseHistoryTest` exercises actual server/client replication,
the authoritative capture boundary, dependency budgets, selection revocation,
client-local exclusion, world transfer, deletion and disconnect with PVS both
enabled and disabled, without modifying canonical transforms to render.
