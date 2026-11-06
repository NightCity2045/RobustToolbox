# Spatial views in ordinary UI

`SpatialViewportControl` is an ordinary clipped UI control. Construct it with
explicit renderer and screen-effect limits, attach it to the UI tree, and set its
`IWorldViewProvider`. Content implements this provider to return a currently
authorized, ready `SpatialPerspectiveFrame` or `SpatialSliceFrame` for the requested
physical render size. Null means unavailable. The descriptor identifies the
existing world; it does not create entities, physics or replication subscriptions.

Perspective descriptors contain the camera and immutable render selection. Match
camera aspect to the requested size. Slice descriptors contain a selected canonical
frame/layer, current frame pose, geometry snapshot, palette, existing sprites and
diagnostics. Match slice size to the request. Each descriptor carries an ordered
list of content-owned vision shaders. Camera controllers, admin/ghost rights,
player attachment, subscriptions, crosshairs and input contexts remain in content.

Each control owns its renderer, slice target and effect chain. Multiple controls
are independently bounded. MaximumVisuals also bounds the combined slice solid
parts, entities and diagnostics; exceeding it rejects the complete selection.
Multiple controls
can observe the same state independently. Switching modes frees the previous mode's
resources; targets are reused until resized. Hiding, removal from the UI tree,
provider replacement and ReleaseResources release resources. ReleaseResources may
also be used on world/round teardown. UI size/scale changes invalidate presentation;
the next draw requests the new physical dimensions. ResolutionScale is a scalar
in (0,1], affecting render resolution while preserving display size and projection.

Unavailable, uploading or over-budget frames draw black and expose neither picking
nor screenshots. On rights/readiness changes call InvalidatePresentation immediately
before accepting further input; the provider must also stop returning the revoked
frame. This does not promise to erase information already seen by a modified client.
Do not substitute an earlier frame or bypass mandatory vision effects when budgets
are exhausted. LastStatus is null before presentation or when no frame is available.

## Coordinates, input and capture

TryGetRay and TryGetSlicePoint accept absolute window pixel coordinates, including
the control's offset. They use the completed displayed projection, not a newly
sampled camera. Perspective rays include near/far distances. Slice points use the
selected frame's full pose and section height. TryProjectWorld provides the reverse
mapping. Outside-control, resized, hidden and unavailable views reject these calls.
Both methods return the canonical world identity for subsequent authorized queries.
Screen coordinates are physical pixels, not UI logical units; do not multiply
them by UIScale again. These are geometric projection methods; content effects that
warp image coordinates must define their own input compensation.

The control intentionally does not implement the planar IViewportControl contract:
perspective input is a ray, not a unique XY map position. Content handles normal GUI
key/mouse events and applies its reachability/interaction policy. It must preserve
UI consumption and focus behavior rather than forwarding every click as gameplay.

TryCopyPixelsToMemory captures the completed view including vision effects at its
render resolution, excluding surrounding UI. It returns false while unavailable.
Ordinary window screenshots include the control and existing UI composition.
The provider owns shaders and input resources; remove/replace their descriptors
before destroying those resources. No target texture is exposed by the control.

## Qualification

The real GL spatial_render_probe renders two ordinary controls through the existing
UI manager at scale 1.5, with different render resolutions, then switches modes
twelve times while alternating UI scale 1.5/2. It checks target counts, full-height
tilted-frame picking, perspective round trips, UI markers, screenshots with vision
effects, revocation, hidden/oversized frames and cleanup on removal. It does not
establish B03 gameplay integration or hardware performance acceptance.
