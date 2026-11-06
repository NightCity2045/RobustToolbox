# XYZ audio backend

`IAudioSource.Position3` and `Velocity3` pass all three world coordinates directly
to OpenAL, in metres and metres/second. Global sources remain listener-relative.
The XY compatibility setters preserve the Z component; use the XYZ setter to
explicitly change height or vertical velocity. Invalid XYZ values throw before
changing native state. Mono positional-source requirements remain unchanged.
Buffered sources use the same XYZ implementation.
Repeated source disposal is harmless; disposal from another thread queues native
release on the audio thread, as for buffered sources.

`AudioComponent` forwards these values to its audio source. They are derived
backend state, not additional replicated entity positions. Canonical source poses
still come from Transform and physical point velocities from the owning body/frame.
Headless sources retain independent state and validate the same XYZ inputs.

`IAudioManager.SetSpatialListener` takes a complete `SpatialAudioListener`: a
canonical-derived rigid pose and velocity. Local +Y is forward and +Z is up;
pitch, yaw and roll all reach OpenAL. The planar listener's artificial Z offset
does not get added to physical height. All inputs validate before native changes.

The selected view adapter can set `AudioSystem.SpatialListenerProvider`, which is
sampled on every audio update. This avoids a subsequent ordinary Eye update
overwriting the spatial listener. Returning null restores the existing flat Eye,
including its configured listener offset. CurrentSpatialListener exposes the
accepted input to the sound adapter. The provider must be removed with its owner.

The listener provider does not replace content propagation or recipient policy.
The spatial build adapter supplies source positioning, XYZ distance/obstacle/room
portal behavior through the existing ProcessStreamOverride hook, using its current
authorized world/view context. The unchanged planar occlusion routine is not a 3D
acoustic model. The ordinary engine source updater explicitly writes planar Z=0
and clears velocity when a source no longer has a physical parent.

## Qualification

Run the explicit SpatialAudioSourceTest fixture with ALSOFT_DRIVERS=null. It uses
real OpenAL Soft source/listener state, reads back XYZ values and full orientation,
checks independent and buffered sources, rejected nonfinite inputs, component
forwarding, view-provider updates and return to flat Eye. No output device is
required. CI runs this fixture separately on Linux and Windows; ordinary headless
tests additionally verify state isolation and matching API validation.

These checks qualify backend coordinates, not the content acoustic/portal model,
local-chat delivery, or reference-machine audio/performance acceptance. Those
remain B09/B11 integration work.
