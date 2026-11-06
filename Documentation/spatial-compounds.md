# Anchored compound ownership

`SpatialMotionOwner.AnchoredCompoundChild` makes a body part of the nearest physical
ancestor through the existing Transform hierarchy. Intermediate nonphysical entities
and nested anchored parts are supported. An independent dynamic body or motor starts
its own ownership subtree: walking passengers and free cargo are not welded into the
vehicle. A capsule motor cannot own structural compound children. No additional
authoritative parent or pose table is introduced.

The root has one backend body containing its own shapes and all active anchored
parts. Member-local collider poses are composed into root coordinates. Each member
contributes its configured mass, distributed across its parts by their mass weights;
the backend computes the combined center of mass and inertia. The root owns motion,
contact solver material and rotation policy. Part entity/surface identity, revisions,
collision layers and masks are preserved. Broadphase uses layer/mask unions, followed
by exact child filtering in contacts, ray/shape/manifold queries and motor support.
An explicit physical root must retain its own nonempty physical definition; content
decides how destruction retires or replaces that root.

An anchored component stores zero independent linear/angular velocity. Its actual
point velocity is derived from the root once, including the angular contribution.
`GetVelocityAtWorldPoint` resolves that owner. `ApplyImpulse` to an active anchored
part applies world momentum at the requested point to its root and publishes the
root's canonical velocity change. Static roots cannot receive dynamic impulses.
Directional interest preloading likewise skips anchored velocity fields and uses
the independent root's world motion.

`ReconfigureBody` can attach a currently independent child or release an anchored
child while retaining its Transform parent. Before release, content can obtain its
world point velocity and add its chosen relative velocity. Release accepts explicit
world linear/angular velocities; attach accepts zero independent velocities. All
affected root shapes and states are prepared before old bodies are removed and
the canonical owner changes. Existing root-origin velocities are preserved; there
is no implicit gameplay momentum, placement, authorization or construction policy.
After release the ordinary Transform APIs may change the independent body's parent.
Directly reparenting an anchored child to a world without a physical ancestor is
rejected before the hierarchy changes.

Lifecycle and movement changes rebuild affected derived roots, including the old
root after a part leaves its subtree. Container dormancy excludes contained parts.
The default container exit retains an anchored part's existing physical root and
world pose; changing it into free cargo requires explicit release first.
Deleting a part removes its mass/shapes; deleting a hierarchy or transferring it
between active physical worlds removes old backend bindings. Independent passengers
retain world poses when a parent moves/rotates instead of receiving the displacement
twice. Their subsequent local poses are derived from the canonical world solve.

Geometry transactions group changed frames by physical root and supply every changed
child snapshot together. One root receives one complete prepared hull. Captured
component identities, revisions, membership and member-relative poses are rechecked
before commit. A stale part rejects the transaction. A whole-root movement does not
change the authored member-relative geometry; commit uses the current root state.
All canonical frames and replacement bodies are installed before completion events.

Constraints may name anchored seats/parts. Engine binding resolves the physical roots
and converts entity-local anchors and fixed relative poses into root coordinates.
Hull rebuilds invalidate old backend IDs and rebuild connections at the next physical
boundary. Two endpoints in the same physical root need no additional connection and
are rejected during connection configuration. Reconfiguration rejects a change that
would merge a live connection's roots or remove its required dynamic participant.
Deleting a named part removes the corresponding constraint component through the
existing lifecycle. For a multi-root world change, `TransferGraph` validates and
moves the complete selected physical graph, including separately parented links;
see [spatial-transfer.md](spatial-transfer.md). Ordinary individual reparenting
retains same-world/dormancy checks and does not choose a docking/destruction policy.

History captures anchored geometry frames even when the root uses explicit colliders.
Historical anchored body state derives point velocity from the captured root and
member pose; it does not read the current hierarchy. Geometry revision expectations
and historical input frames therefore retain the original child identities.

Tests cover per-part masks in actual contacts and query/motor paths, center-of-mass
impulses, combined mass removal, anchored movement versus independent passengers,
world transfer/deletion, paired child-geometry commits and stale pose rejection,
attach/release/re-attach, seat constraints across hull rebuilds, and historical child
geometry/state. Map save/load and cloned graphs rebuild combined mass, remap part
collider sources and seat endpoints, and restore the connection. A connected client
receives anchored ownership/local pose and part deletion without a second solver.
Full moving-structure acceptance still needs content controllers,
multi-client moving-vehicle scenarios and performance
qualification; these tests alone do not close E08/G8.
