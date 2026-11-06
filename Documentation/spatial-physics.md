# Trusted spatial physics adapter (E03, in progress)

`ISpatialPhysicsBackend` is currently an internal engine boundary. The BEPU
implementation owns its simulation, pooled buffers, compound indices and body
handles. It accepts neutral body definitions and returns neutral body states and
source-qualified ray hits. It is not yet registered with ECS or enabled for game
entities. World/frame-aware query policy, movement ownership, motor, constraints,
contact events, prediction and replication remain required integration work.

Static, dynamic and kinematic bodies use compound box, sphere and Z-axis capsule
parts. Each part retains optional Entity/Surface references and a geometry
revision. Ray queries apply a caller-supplied mask and optional ignored body.
Collision masks must permit both sides of a contact. Physical coefficients are
engine-neutral values, separate from content prototype/material identifiers.
Kinematic velocity integration alone does not implement safe door/platform motion;
the controller must supply checked movement through the upcoming sweep/motor API.

Dynamic compounds are recentered for correct inertia. The adapter converts poses
and linear velocities at the canonical entity origin to/from the backend center
of mass, including angular velocity crossed with the origin offset. No content
position table or public BEPU handle is introduced. Invalid initial/state inputs
are rejected before changing a live body. Removing a body also disposes its
compound and child shapes; opaque internal IDs are never reused.

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
