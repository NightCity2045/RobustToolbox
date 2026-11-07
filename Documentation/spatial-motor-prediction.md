# Local spatial motor prediction

`SpatialMotorPredictionSystem.PredictMotor(entity, bounds, collisionLimits,
maximumPredictionTicks)` selects a known server capsule for local prediction.
Content owns the selection, input event type, permission checks, movement modes,
parameters and disposal. One selected motor is allowed per world. The handle's
status reports whether prediction is active, waiting for a complete state,
outside received coverage, inactive, constrained, or beyond its tick budget.
Changing bounds requires a new authoritative capture and does not grant PVS.

The driver runs after shared spatial physics and geometry replica installation.
It reuses `ClientGameStateManager`'s input sequence acknowledgements, predicted
component reset and pending input/event replay. It does not keep another input
queue. Every authoritative state invalidates the speculative solver; explicit
prediction reset does too. A fresh solver starts at that state's tick, with the
matching complete geometry revision. It cannot resume halfway through a replay
by skipping earlier inputs when collision data arrives late.

The shared motor intent/settings, velocity, impulse and capsule resize APIs accept
client writes only for an explicitly selected, active motor during prediction.
They dirty the existing components so the ordinary authoritative reset restores
them before replay. Resize uses the same foot-preserving capsule replacement and
overlap clearance as the server; missing clearance returns false. Inputs can
arrive between physics ticks as well as during replay. Gameplay must tolerate a
predicted resize refusal and accept the subsequent authoritative result.

Motion uses the same BEPU backend, world gravity/substep settings, motor impulse,
support, slope, checked stair and ceiling algorithms as server physics. Successful
steps commit into the existing Transform and SpatialBody, with a predicted support
witness in SpatialMotor. There is no second Entity or canonical position store.
Predicted solver contacts never raise authoritative landing, impact or damage
notifications. Content remains responsible for idempotent predicted presentation
effects and authoritative gameplay consequences.

Remote colliders use captured server poses and velocities as private kinematic
proxies. Anchored compound members share one root proxy, with their original
source identities, materials, masks and mass distribution. Rotation therefore
supplies support velocity at the contact point, including `omega cross r`, without
moving canonical remote entities or carrying the passenger twice. Remote proxies
do not predict other players' inputs or remote dynamic reactions. These are
server-corrected approximations, not lockstep physics. Point connections and
distance limits attached to the motor use the captured constraint definitions.
Anchors on compound children resolve through their captured local hierarchy to
the physical root. Capsule replacement restores these connections before the
next step. Missing endpoints prevent prediction rather than dropping a constraint.
Fixed connections still wait for authority: a rotation-locked motor and a remote
kinematic proxy cannot supply the angular inertia that a weld requires. Piloted
dynamic-body prediction is a separate remaining integration.

Unknown collision never becomes free space. The complete collision view must be
ready at the captured revision. Every motor sweep/overlap, support search and
clearance query must fit its certified bounds. A scoped backend check also observes
the capsule at every solver substep and after final integration. The locked
capsule's swept volume lies inside the convex hull of these endpoint bounds, so
leaving and returning during one tick cannot evade the check. If any check fails,
the entire speculative solver is discarded and no pose/support from that step is
published. The boundary adds no fictitious wall, floor or impact event. Prediction
holds its last safe pose until a new authoritative state can be replayed.

Disconnect, entity/world deletion, transfer, flush and explicit handle disposal
release the solver and collision view. Selection and tick/entity/shape budgets
bound retained state. This API supplies no production budget defaults or hardware
SLO claim. Read-only authoritative interpolation remains a separate mechanism;
content view integration owns presentation of its controlled character and visual
correction smoothing.

Qualification covers connected client/server input replay and server rejection,
walls, foot-preserving resize, impulse-driven jumping, no client landing
consequences, unavailable coverage, bounded prediction, dropped states, container
dormancy and moving compound support. Backend tests include an excursion that
returns inside the box before the tick ends.

The same network scenario runs with nominal RTT profiles of 100 and 200 ms,
10 ms one-way jitter and every fifth unreliable packet dropped, including input
ACK/state traffic. The test transport delays the actual serialized messages on a
shared virtual clock independent of prediction rewinds and machine speed.
At 60 Hz it measures each delivered delay and checks the configured bounds plus
one tick of dispatch quantization, verifies actual drops, and verifies reliable
ordering and bounded queue cleanup. Reliable delivery remains the transport's
contract. This is reproducible protocol qualification; OS UDP/retransmission,
multiple-player gameplay, vehicle controls and idempotent content effects still
need their respective integration gates.
