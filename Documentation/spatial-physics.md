# Trusted spatial physics adapter (E03, in progress)

`ISpatialPhysicsBackend` is currently an internal engine boundary. The BEPU
implementation owns its simulation, pooled buffers, compound indices and body
handles. It accepts neutral body definitions and returns neutral body states and
source-qualified ray hits. `SharedSpatialPhysicsSystem` connects this backend to
explicit spatial world profiles and ECS bodies. The ordinary content maps and
prototypes have not been migrated or enabled by this engine change. Motor,
client collision streaming
and prediction remain required integration work.

Static, dynamic and kinematic bodies use compound box, sphere and Z-axis capsule
parts. Each part retains optional Entity/Surface references and a geometry
revision. Ray queries apply a caller-supplied mask and optional ignored body.
Collision masks must permit both sides of a contact. Physical coefficients are
engine-neutral values, separate from content prototype/material identifiers.
Kinematic velocity integration alone does not implement safe door/platform motion;
the controller must supply checked movement through the upcoming sweep/motor API.
The current ECS activation path accepts Static and DynamicBody owners. Motor and
AnchoredCompoundChild definitions require their forthcoming integrations and fail
explicitly if activated prematurely.

Dynamic compounds are recentered for correct inertia. The adapter converts poses
and linear velocities at the canonical entity origin to/from the backend center
of mass, including angular velocity crossed with the origin offset. No content
position table or public BEPU handle is introduced. Invalid initial/state inputs
are rejected before changing a live body. Removing a body also disposes its
compound and child shapes; opaque internal IDs are never reused.

## ECS, persistence and authority

`EnablePhysics` activates an existing SpatialWorld map after checking its hierarchy.
One backend is created per active world. `SpatialBodyComponent` stores a neutral
collider/mass/filter definition and world linear/angular velocity at the entity
origin; the existing Transform remains the only entity pose. Explicit colliders
or the frame's canonical geometry supply shapes. Geometry parts retain source
Entity/Surface/revision and use volume-proportional mass weights. Frames with only
canonical geometry become static colliders in an active world.

Planar physics bodies cannot enter an active spatial world, and an entity cannot
have both physics components. Reparenting checks the whole subtree before changing
hierarchy or anchoring. Bodies can transfer between active worlds or become dormant
in nullspace. Attached structural geometry under another body requires compound
ownership rather than a separate infinite-mass static collider; independent dynamic
bodies can remain children of a moving frame.

Each server tick steps the world backend and commits complete world poses through
`SharedTransformSystem.CommitSpatialPoses`. Local poses are derived from all final
parent poses, including intermediate frames. All poses and velocities are installed
before move notifications. Independent bodies therefore do not inherit their
parent's movement a second time, including paused children. This batch does not run
legacy planar grid traversal; spatial support/frame selection belongs to the motor.

Containers suspend bodies while retaining canonical velocities; removal creates one
body in the destination world. Pausing freezes a body as a collider and preserves
its velocity for resumption. Body/frame/world deletion releases backend resources.
Map saves store the world profile, neutral definitions and velocities; load/clone
rebuild handles and source references. Collider rotations use the same quaternion
format as canonical transforms. No backend indices are saved.

Network component states carry neutral definitions/settings and velocities; poses
use the existing XYZ transform states. A connected client receives these states
but does not run a second authoritative solver. Client prediction and streamed
collision baselines remain E07. Initial body definition availability is explicit.
Geometry is still not sent as an unbounded ordinary component state.

## BEPU 2.4.0 continuous collision corrections

Qualification of isolated convex bodies did not cover the full compound path.
The new adapter tests reproduced a compound crossing a thin wall at 300 m/s,
including a direct dependency-only reproduction. The raw failing result is kept
in task evidence; this is not evidence that the original dependency passes.

In the pinned source, `BoundingBoxHelpers.GetLocalBoundingBoxForSweep` returns
relative displacement multiplied by `maximumT`. Both compound overlap finders
then pass that displacement and `maximumT` to `FindLocalOverlaps`, which scales
travel a second time. At a 1/30 second step, the candidate search covers only
1/30 of the actual displacement. `SpatialSweepRegistration` registers corrected
finders through BEPU's supported task registry: traversal uses [0,1], while
angular expansion and actual narrowphase time-of-impact retain physical seconds.
The package version and sandbox permissions are unchanged. Supported registrations
cover Compound/BigCompound pairs and box/sphere/capsule versus those compounds.

A separate rotational case found insufficient broadphase bounds for offset
children. BEPU 2.4.0's compound bounds batcher clamps child tangential velocity
against an offset radius, missing fast angular travel. The adapter uses the
supported post-prediction/pre-collision stage to enlarge active rotating compound
bounds conservatively. A point within radius R moves at most
min(|omega|*dt*R, 2R) due to rotation. This bound is combined with translation and
gravity displacement; existing predicted bounds are retained. Narrowphase still
checks the actual shapes. This does not change tickrate or disable contacts.

Source references (Apache-2.0):

- https://github.com/bepu/bepuphysics2/blob/v2.4.0/BepuPhysics/BoundingBoxHelpers.cs
- https://github.com/bepu/bepuphysics2/blob/v2.4.0/BepuPhysics/CollisionDetection/SweepTasks/CompoundPairSweepOverlapFinder.cs
- https://github.com/bepu/bepuphysics2/blob/v2.4.0/BepuPhysics/CollisionDetection/SweepTasks/ConvexCompoundSweepOverlapFinder.cs
- https://github.com/bepu/bepuphysics2/blob/v2.4.0/BepuPhysics/Collidables/Compound.cs

Performance and the complete E03/G3 acceptance remain unqualified. Conservative
bounds can increase candidate counts; they are required for correctness and must
be measured together with the final ECS/controller integration.

## World/frame-aware queries

`SpatialQuerySystem` implements `ISpatialQueryService`: raycast, shape cast,
overlap, free-volume checks and directional semantic-surface search. Each call
requires a `SpatialQueryContext` with world, input frame, purpose and mask.
Input coordinates/displacements are frame-local; returned points/normals are
world-space. Rotation uses a world-axis angular displacement after converting
from the input frame, over the same normalized [0,1] interval as translation.
A rotating cast's distance measures center translation, while its fraction
identifies progress of the complete translation/rotation.

The caller can filter source entities/surfaces/material profiles according to
its purpose. The engine supplies mechanism, not gameplay permission or one
shared permeability rule. Ray filtering happens during child traversal, so a
rejected nearer candidate does not hide an eligible farther one. IgnoredEntity
excludes matching source entities. Filters must be read-only; recursive queries
are rejected. All sources retain semantic surface revision and material profile.

Queries traverse the world's broadphase and each compound's child tree, then use
BEPU convex narrowphase for the actual shape pair. Overlapping bounding boxes
alone do not produce an overlap result. Box, sphere and Z-capsule casts support
translation and rotation, including pure rotation outside the initial bounds.
A sweep against the current scene does not predict other bodies' future motion;
backend continuous collision handles relative motion during physical simulation.

An initial overlap has no unique sweep contact witness: Point/Normal are null,
Fraction/Distance are zero, and InitialOverlap is true. Callers must explicitly
handle this condition rather than treating the default vector as a surface normal.
Overlap and IsVolumeFree conservatively include touching shapes.

Optional expected frame revisions are checked before querying. Wrong-world,
stale/missing geometry and unavailable collision scenes throw; they never report
empty space. Client collision streaming/prediction remains pending, so a client
without its collision baseline cannot use this service to authorize movement.
Contained bodies remain excluded from the world's active collision scene; content
uses its container access rules separately. Query results do not grant access to
items inside a container.

## Impulses and connections

`ApplyImpulse` accepts world-space momentum and an application point for an active,
unpaused dynamic body. BEPU applies mass and transformed inertia at the center of
mass; canonical velocities remain measured at the entity origin. Invalid or
unrepresentable results are rejected before solver mutation. Zero impulse has no
side effects. `SpatialImpulseApplied` is raised once on the server after component
velocities are committed; content determines any gameplay consequences.
`GetVelocityAtWorldPoint` evaluates the canonical linear + angular-cross-offset
velocity. Paused/contained bodies retain canonical velocities for resumption; this
utility does not imply that a dormant body is currently moving in the solver.

`SpatialConstraintComponent` stores a neutral, serializable connection between two
entities. Supported connections are fixed relative pose, a point attachment with
free angular motion, and a minimum/maximum distance range. Anchors are relative to
entity origins, with center-of-mass offsets accounted for inside the adapter.
At least one participant must be dynamic. Spring frequency/damping and whether
connected participants collide are explicit parameters. Joint-specific gameplay
rules, including seat belts and pulling permissions, remain in content.

Static participants use private shapeless kinematic anchors because BEPU joints
connect body handles. These anchors are derived solver data, follow static pose
changes, can sleep, and are removed with the connection. Multiple connections
reference-count collision suppression; deleting the last one restores contacts
and wakes the affected islands. Stale constraint IDs cannot alias new links.

Initial constraints are bound at the fixed tick boundary after participant
startup. Runtime replacement prepares its new connection before replacing the
canonical definition. Inactive/container participants, paused connection entities,
and partial world transfers suspend the solver connection. Moving the whole
subgraph to another physical world rebuilds it there. Deleting a body removes
referencing constraint components without deleting the content-owned link entity.
Map save/restore/clone remaps both participants; no backend handle is serialized.
Network states use NetEntity references and initialize client definitions without
starting a second authoritative solver. Client prediction remains pending.

## Contact transitions

`SpatialContactStarted` and `SpatialContactEnded` are authoritative directed
notifications after the tick's canonical poses and velocities have been committed.
Each includes both semantic sources, material profiles/revisions, world point,
normal toward the recipient, penetration depth and pre-solve relative point
velocity. An end retains its last touching witness; it does not describe a new
impact. Game damage and support/landing policy are not implemented by these events.

BEPU child callbacks run before nonconvex reduction adds child offsets. The adapter
converts witnesses using the actual rotated compound-child origin and accounts for
center-of-mass angular velocity. New speculative proximity alone does not start a
touch. Existing contacts have a 0.1 mm separation hysteresis to avoid transitions
caused by numerical roundoff at rest. Contact retention is initialized after BEPU's
sleep stage, so a body falling asleep during this tick does not end its contacts.

The ECS layer aggregates convex-part transitions by world and semantic source pair.
A seam in one slab therefore cannot produce duplicate gameplay impacts. Transient
start/end pairs remain observable, including when a move handler removes a body
before notifications. Event delivery snapshots its work so handlers can delete a
body or world safely. World teardown releases the semantic contact registry.

Pausing a dynamic body now preserves its solver identity and shape, freezing it
through zero inertia/velocity. Canonical velocities remain available for resume.
Eligible connections are rebuilt at the fixed boundary after the inertia change.
Existing frozen contacts against static geometry persist until geometry changes;
sleep, pause and resume do not manufacture a fresh collision. Queries continue to
see the frozen collider. Backend and ECS tests cover sleep/pause, surface seams,
rotated offset parts, deletion from event handlers and 300 m/s thin-wall contacts
at both 30 and 60 Hz.

`OverlapContacts` additionally returns detached convex manifold witnesses with
world normals and penetration depths. It distinguishes a touching support from an
actual occupied volume for resize/depenetration consumers. The adapter selects
candidates through the same shape-query path, then runs BEPU's collision batcher
without inserting a temporary body, stepping the simulation or emitting gameplay
contacts. Internal compound child indices retain source identity across batch
flushes. The existing sweep InitialOverlap result remains explicitly witness-free;
callers request a manifold when they need penetration information. A manifold
normal is a solver-selected separating direction, not a claim of a unique witness
for completely overlapping shapes.
