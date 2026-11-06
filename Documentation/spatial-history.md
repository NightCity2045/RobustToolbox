# Coherent physical history

`SpatialHistorySystem` records explicitly configured server worlds at the end of
the spatial physics step, after geometry transactions, canonical pose/velocity
commit and support/contact publication. The timestamp is that engine `GameTick`.
Repeated explicit steps at one tick replace its capture. The snapshot describes
that physical boundary; it does not claim to include gameplay changes made later
in the server tick.

Content calls `Configure(world, SpatialHistorySettings)` to choose the retained
tick window and aggregate pose/shape sample budgets. History has no implicit lag
compensation policy or default window. A budget can shorten the time window.
An individual capture that cannot fit produces `SpatialHistoryGap` and is absent,
not a partial or empty scene. Age eviction also runs after a failed capture.
These sample limits are conservative allocation bounds, not measured RAM/SLO
qualification. Content still needs to select and qualify its production budgets.

`TryGetScene(world, tick, out scene)` resolves only that exact captured tick. There
is no nearest-tick fallback or invented interpolation. A scene contains active
collision bodies, their canonical world poses/velocities and definition versions,
the poses of their ancestors/source entities, and immutable geometry snapshots
for physical geometry frames. Contained/dormant bodies are absent, matching the
live physical scene. Arbitrary nonphysical frames are not implicitly recorded;
`TryGetPose` and frame-qualified queries explicitly report their absence.

Each scene implements `ISpatialQueryService`: ray/surface queries, translational
and rotational shape casts, overlaps, manifolds and volume checks use the frozen
body definitions and geometry from the same tick. Input coordinates use captured
frame poses, never the current hierarchy. Expected geometry revisions must match
the historical snapshots. Witnesses are returned in historical world space, with
historical source/surface identities and material revisions. Content supplies the
query filter and decides whether an identity that has since been deleted can
receive an effect. No engine query decides a hit, damage or latency allowance.

Queries build a private static collision scene lazily. At most one such backend
is retained by the history system across all worlds/ticks; switching scenes
releases the previous backend. It is never stepped and cannot rewind or publish
state into the live ECS/solver. Recursive history queries and reconfiguration
inside a query policy are rejected. Filters retain the common read-only contract.
Unchanged immutable geometry is shared across captures by component identity and
revision. Recreating a component cannot reuse a snapshot from its former lifetime.

Eviction, reconfiguration, disable, physical-world removal, world deletion, flush
and shutdown invalidate old handles and release derived resources. `IsAvailable`
becomes false; pose/body/geometry lookups fail and queries on the expired scene
throw. A consumer holding a handle does not extend the history window.

Tests cover simultaneous body motion and geometry edits, historical frame poses,
query/manifold results, stale revision rejection, deletion after capture, shared
unchanged snapshots, missing ticks, tick/budget eviction, impossible capture
budgets, disabled history, physical-world removal and recursive query guards.
Content lag-compensation policy, client reconciliation and replay remain separate
integration work under E07/B05/B11.
