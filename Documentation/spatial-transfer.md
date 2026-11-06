# Closed spatial graph transfer

`SharedSpatialPhysicsSystem.TransferGraph(roots, destination, worldMapping)` moves
explicitly selected disjoint Transform hierarchies between existing spatial physics
worlds. The mapping converts source world poses into destination world poses. It
rotates world linear/angular velocities and motor intent exactly once; it does not
add a second parent or support velocity. Entity IDs, layers, geometry revisions,
local child poses, containers and internal references remain canonical and retain
their identities. No entities or contents are cloned.

Content chooses the roots, including independent passengers, cargo or a separately
parented connection entity. Descendants and contained entities are included through
the existing hierarchy. Both endpoints of every affected spatial constraint and its
connection entity must be included. Incoming and outgoing geometry source references
must also be closed. An incomplete selection is rejected before mutation; the engine
does not silently drop a connection, move another vehicle, or invent a docking/FTL
policy. Content must validate its own atmosphere/network/portal/gameplay links before
calling this technical mechanism.

Preflight requires running disjoint hierarchies in one source spatial world, a
distinct destination with active spatial physics, and representable complete poses
and velocities. A contained object cannot be a top-level root; include its container
hierarchy or explicitly release it first. Anchored parts require their physical root.
Legacy planar grids must be migrated to spatial frames before using this operation.
Nullspace detachment and ordinary individual reparenting retain their existing APIs.

Destination body shapes, inertia, states and solver connections are prepared before
canonical changes. Preparation failures dispose destination allocations and leave
source bodies and hierarchy intact. The commit installs all parents, map membership,
pause states, rotated canonical velocities, body bindings, connections and motor
supports before map/pause/move notifications. Independent passenger bodies move with
the world mapping once. Contained bodies remain dormant. Entering a paused world
preserves canonical velocities while pausing derived bodies; returning to an active
world restores simulation and its connections.

A motor and its transferred support retain a transformed support witness. If just
one side moves, the old support is cleared on either side of the world boundary.
Support changes are published after commit without fabricating a landing impact.
`SpatialGraphTransferred` publishes the selected entity set and mapping after the
ordinary lifecycle notifications. Reentrant transfers or simulation steps during the
operation are rejected. As with other committed ECS changes, notification-handler
exceptions do not roll back an already committed world change.

The ordinary entity state stream carries the changed hierarchy and canonical state.
Geometry streaming invalidates cached collision immediately when a reparent changes
world membership, even if the geometry lifetime/revision stays identical. A coherent
manifest for the new world must resolve its dependencies before collision coverage is
ready again. Historical scenes remain snapshots of their original world and tick.

Tests cover multi-root compound/seat/passenger transfer, container contents, rotated
velocities, paused destination and return, observer consistency, rejection of split
connections, overlapping roots, incoming/outgoing geometry references and overflowing
descendant poses, motor support on both sides of a transfer, actual connected entity
replication, and geometry-cache invalidation/reinstallation over the network. Content
transport controls, atmosphere/docking policies and multi-client gameplay acceptance
remain separate work in B07/B08 and G8.
