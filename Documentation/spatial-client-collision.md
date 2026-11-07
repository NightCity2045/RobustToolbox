# Client collision views

`SpatialCollisionViewSystem.CreateView(world, bounds, limits)` creates a disposable
read-only view of authorized replicated collision. Content owns its world-space
query bounds and budgets for entity visits, retained entities and convex parts.
Creating or moving a view grants no PVS subscriptions and does not extend the
server's geometry coverage. There is no implicit production budget or claim of
hardware performance qualification.

The system captures canonical full poses and copied body definitions after
`GameStateApplied`, at the applied state tick. Client-created entities and
uninitialized/detached entities are excluded. A missing parent or body definition
prevents a complete capture. Ancestors count against the entity budget. A query
cannot capture live predicted transforms merely by requesting a view.

`TryGetScene` succeeds only when the authorized geometry replica certifies the
entire requested box. Geometry revisions and component generations must match
the captured state. Pending collision changes, invalidated, absent or over-budget collision return
no scene. It does not produce an empty scene on which a character could walk.
Only a completed, certified scene may legitimately answer that a volume is free.

The returned `SpatialClientCollisionScene` implements `ISpatialQueryService` with
the same frozen-query implementation used by server physical history. Rays,
surface rays, translational/rotational sweeps, overlaps and contact manifolds use
captured XYZ poses, shape definitions and semantic surfaces. Compound members
retain their source identities and masks. Contained bodies and their descendants
are dormant. The private backend contains static query representations and is
never stepped; no canonical ECS pose, velocity, body or geometry is changed.

Query contexts name a captured coordinate frame, purpose, collision mask, optional
ignored entity/filter and expected geometry revisions. Results are world-space
witnesses. A ray's whole segment must lie inside certified coverage; a swept
shape is conservatively enclosed by its bounding sphere over the translation,
including every intermediate rotation. A query outside that box fails instead
of silently clipping its path or treating the unobserved remainder as clear.

Every new authoritative state retires the old scene. Pending/new geometry also
makes old scene handles unavailable. Changing bounds requires a new state capture;
world transfer, deletion, entity flush and disconnect invalidate or dispose the
view. Calls belong to the client thread, recursive query callbacks are rejected,
and disposal during a callback defers backend release until the query exits and
rejects its now-obsolete result. Content must dispose views when their users close.

A pending update that only moves authorized coverage and reuses every complete
geometry frame can retain queries inside the intersection of the old and new
coverage. It neither exposes newly requested space early nor retains revoked
space. Added, removed or changed geometry and full recovery baselines still block
queries until atomic installation at the corresponding authoritative tick.

These are queries over the client's authorized knowledge, not server hit decisions
or permission to reveal hidden entities. Server gameplay validation remains
mandatory. The view establishes a collision-consumption boundary. The separate
[local motor driver](spatial-motor-prediction.md) consumes the same coherent
capture under the existing game-state reset and input replay boundary.

The network fixture checks both PVS modes, body/surface masks and witnesses,
canonical pose preservation, entity/shape budgets, refusal of unauthorized bounds,
a real multi-fragment geometry replacement, contained body removal and disconnect.
During replacement it observes the unavailable interval before the new geometry
can be queried. Server historical-query regressions exercise the shared query path.
