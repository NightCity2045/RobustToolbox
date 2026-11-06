# Spatial rendering

`IClyde.CreateSpatialRenderer` creates an independent perspective view in the
existing Clyde GL context. Composite `renderer.Target.Texture` through normal UI
drawing; no additional window or renderer process is involved. Each view owns its
target and GPU mesh cache. Dispose it on view/world removal. Resize uses physical
pixels and prepares all replacement targets before releasing the old ones.

The currently qualified capability profile is desktop GL with VAOs, sampler
objects, floating-point framebuffers and sRGB. Query `SupportsSpatialRendering`;
headless Clyde returns false. GLES and target gaming-hardware performance are not
qualified by the Linux llvmpipe functional capture.

## Input and authority

`PerspectiveCamera` is an immutable eye pose, FOV, aspect and near/far projection.
Local +Y is forward, +X right and +Z up. Picking uses the same projection, including
the true clip distances at the screen corners. Frustum tests accept local chunk
bounds plus the canonical frame pose, including tilted moving structures.

`SpatialRenderScene` accepts an already authorized selection of immutable meshes,
frame poses, explicit material bindings and XYZ point/spot lights. It does not
instantiate entities, grant visibility, run physics or own authoritative poses.
Read current frame poses from Transform on each draw. Linear material color and
emission, cutout threshold and repetition are explicit appearance inputs.

The render passes are opaque/cutout with depth writes, two depth-tested weighted
transparency passes, and composition. Weighted blending approximates intersecting
transparency; it is not refraction. Forward selection retains at most 16 lights per
chunk. Alpha-cutout geometry remains in the depth-writing pass. Mesh source IDs,
surface revisions and material profiles survive batching for diagnostics/picking.

Raw GL state is saved/restored around spatial work. Queued 2D drawing is flushed
before borrowing the context. ReloadShaders compiles all replacement programs
before exchanging any of them, retaining the working programs on failure. The existing
resource watcher also handles spatial `.vert`/`.frag` edits.

## Bounded shadows

Mark selected point/spot lights with CastsShadows and configure
`SpatialRendererLimits.Shadows`. The desktop uniform budget permits at most four
shadowed sources, each with six perspective faces. Resolution, near plane and
depth bias are explicit quality inputs. The complete atlas must fit MaximumPixels;
this limit includes all six faces per light. Zero lights disables this pass.

Opaque and alpha-cutout geometry writes a nearest-occluder depth atlas. Lighting
uses four clamped depth comparisons per lookup; atlas-face edges cannot sample the
adjacent tile. Transparent surfaces receive illumination but do not cast colored
transmission shadows. Ambient/emission remain independent of direct shadows.
Selection is deterministic by normalized camera distance. Spot illumination keeps
its cone; its shadow projection uses the same six-face representation as point
sources, including wide spot cones.

Shadow caster selection includes authorized geometry outside the main camera
frustum. Its meshes count toward residency and upload limits. Disabling shadows
does not remove any visible mandatory geometry, change physics, or broaden the
authorized input selection. Different shadow quality is not a visibility policy.

## CPU and GPU budgets

`SpatialMeshCache` belongs to one world and one client thread. Request authorized
snapshots, call Pump each frame, and TryGet using the exact current snapshot.
Request immediately hides the old revision. Remove revoked/deleted frames and
dispose the cache when its world goes away. Cancelled/superseded jobs cannot install
into a replacement frame, including a reused UID or revision.

Workers operate only on detached snapshots. Changed surface bounds invalidate XY
chunks and their edge neighbors across authoring layers. Exterior construction
still uses all solids for occlusion. Unchanged chunks share their immutable arrays;
the current snapshot association is replaced. A dynamic structure's pose alone
does not trigger a mesh rebuild.

Choose explicit frame, concurrent job, quad, chunk and resident CPU byte limits.
In-flight memory is additionally bounded by the number of jobs times each job's
quad/chunk limits; resident accounting is not a total-process RAM measurement.
Budget failure never publishes part of a frame. A budget-failed request can be
submitted again after capacity is freed. Inspect GetStatus/error for failures.

GPU limits separately cap mesh residency, upload bytes per Render call and target
pixels. Uploads can span calls. Until every required visible mesh is ready, Render
clears the output and returns Uploading; oversized visible selections report
BudgetExceeded. The caller must handle these states explicitly. Old walls or
missing mandatory geometry are not used as a silent fallback. Target attachments
also consume memory in addition to the mesh byte count.

## Isolated material mipmaps

Build `SpatialTextureMipChain` from the specific source image region, with explicit
dimension and byte limits. Cropping precedes reduction: neighboring atlas sprites
never enter the pyramid. Each pyramid becomes a separate repeating texture, so
its mip levels need no shared-atlas padding. Existing RSI atlases stay unchanged.
Reduction uses linear light, alpha-weighted color and area weights for odd image
dimensions. An optional cutout threshold preserves the nearest representable alpha
coverage without feeding corrected alpha back into subsequent reduction levels.
Building works on detached pixels and supports cancellation without accessing ECS
or GL. The caller must keep the source unchanged during the initial copy.

On the Clyde thread, `CreateSpatialTextureUpload` allocates a complete pyramid
within the supplied per-texture GPU byte limit. Call Upload with a byte budget;
it transfers whole RGBA pixels and can stop partway through a scanline. Texture
access is rejected until every level is ready. Dispose cancels unfinished work or
releases a finished texture. Texture data is immutable; replace the entire pyramid
for an appearance change. The caller also owns aggregate cache/residency limits.

Bind the completed texture with `SpatialTextureSampling.PixelMip`. Magnification
uses nearest sampling; minification interpolates adjacent mip levels, with nearest
sampling within each level. Explicit gradients come from unrepeated UVs so repeat
seams do not select an incorrectly coarse level. Forward and shadow passes share
the same material binding. A cutout material's threshold must match the pyramid;
transparent materials cannot use cutout-corrected alpha. Ordinary `Nearest`
bindings continue to support existing atlas regions with half-texel insets.

## Flat authoring view

`TopDownSliceView` is a separate frame/layer projection. Its explicit frame-local
height cut selects floor footprints and boundary sections from the same semantic
snapshot. Surface picking applies the same layer and hole filtering. Existing
layered entity sprites go through `IRenderHandle.DrawEntity`; diagnostics go through
normal 2D primitives. No perspective billboard is viewed edge-on from above.

The caller supplies a flat surface palette, authorized entities with explicit layer
assignments, and optional diagnostics. The caller also owns target clear/clipping.
Drawing and picking both use physical viewport pixels; convert UI positions with
the same scale. ProjectWorld/UnprojectWorld support rotated and tilted frames.

## Qualification and remaining work

Run `spatial_render_probe` in a graphical client with a started ECS and the engine
debug prototypes available. It writes PNGs and a JSON report in user data. It renders
one semantic scene in perspective and two flat layers, uses an existing directional
RSI sprite, reverses transparency submission, reloads shaders eight times, composes
UI markers, checks layer filtering/openings and exercises target resize/disposal.
It also compares shadows on/off, checks a completely off-camera occluder, and swaps
a growing/shrinking mesh pair at an unchanged exact GPU residency limit.
Material captures check nearest magnification and isolated minification against
known pixels, a 13-byte upload budget, hidden incomplete textures, cancellation
and unchanged texture counts after disposal.
Do not run the qualification loop during a live performance measurement.

The standalone local fixture used `--self-contained`, desktop
`display.opengl_version=1`, a minimal content module registering engine components
and starting ECS, and an Xvfb display. This environment needs SDL3's
`SDL_VIDEO_FORCE_EGL=1` to obtain an sRGB-capable desktop context. This is an
environment-specific launcher setting, not a change to normal game startup.

This implementation does not close E05/G4. Full game viewport/view-switch
integration and broader performance
acceptance remain. E06 perspective asset profiles, composition caches, effects and
XYZ audio, as well as content resource bindings and gameplay integration, remain
separate required work. The diagnostic images are not screenshots of integrated
spatial gameplay.
