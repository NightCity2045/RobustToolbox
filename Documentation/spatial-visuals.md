# Spatial asset presentation

`SpatialRenderScene.Visuals` contains authorized derived draw records alongside
the canonical geometry chunks. It does not create entities or physics shapes.
Quad and Box use shared unit meshes with per-instance pose, physical dimensions,
UV region and material. A primitive assembly is a sequence of these records with
local poses composed under the same canonical owner. It needs no separate scene
graph. Resource/material choices remain explicit content inputs.

Visuals participate in opaque/cutout, weighted transparency, light selection and
bounded shadows, including off-camera casters. Their shared meshes count against
the same GPU residency and upload budget as chunks. MaximumVisuals bounds draw
selection; exceeding it returns BudgetExceeded rather than dropping mandatory
objects. Camera movement never rebuilds or uploads shared primitive geometry.

## Existing sprites and observer direction

`SpatialSpriteProfile` supports DirectionalBillboard, GroundSprite and SurfaceQuad.
Size is in metres; pivot is normalized from the lower-left image corner. Billboard
up follows canonical local +Z and turns toward the individual observer.
MaximumTilt limits steep-view tilt, with stable finite bases at the poles.
Dimensions and canonical entity pose do not change with the observer.
GroundSprite and SurfaceQuad use the supplied pose's XY plane, facing local +Z.
Content supplies attachment/lying/sitting poses and physical body changes.

`SpatialSpriteAnimation.Direction` selects existing 1/4/8-direction RSI images
relative to canonical orientation; local -Y is the south/front image. Frame samples
existing folded delays with explicit time and looping policy. Neither method
advances or rotates a shared SpriteComponent. Resolve each layer's actual texture
with GetFrame, including its own directional offset and animation state supplied
by content. Missing artistic views are not synthesized.

`SpatialDecalAttachment` stores a stable surface reference, revision, face plane,
rectangle and normal offset. TryResolve checks the current exterior snapshot and
composes under the current frame pose. Removal, a revised surface, an opening,
or a rectangle extending outside its exterior patch returns false. The caller
updates/revalidates an attachment after a geometry edit. This mechanism adds no
collision or gameplay ownership to a decal.

## Layer composition cache

Own one `SpatialSpriteCache` per client world/lifecycle and dispose it on teardown.
Supply authorized ordered `SpatialSpriteLayer` records: existing image frames,
source pixel regions, destination pixel rectangles, affine transforms and sRGB
tints. Composition copies this sequence. Its complete structural signature
includes every layer parameter and texture identity, allowing multiple entities
and observers to reuse an identical result. Gear slots, species, held-item rules
and appearance state machines belong to content.

Request returns a lease, or null if the full appearance cannot fit. Pending leases
do not expose a texture. Pump takes an existing render handle and a target-pixel
budget; dimensions and maximum layers also bound each composition. An appearance
larger than the Pump budget remains pending, so budget for at least one complete
allowed target. Ready targets are reused, including when an animation cycles back
to a cached frame. There is no per-entity per-frame render-target allocation.

Entry/byte limits include pending reservations. LRU eviction only removes unleased
entries, and a rejected request does not evict useful entries. Release leases when
the appearance/view changes. Invalidate a mutated, reloaded or lost texture (or
its atlas source) before reuse; this immediately revokes affected active leases.
Clear/dispose invalidates all leases and releases GPU targets. Use IsValid/IsReady
to handle these states; do not retain an unleased Texture for a later scene.

Composition targets store premultiplied RGBA in sRGB textures. They start at
transparent black: Robust's Color.Transparent has white RGB and is unsuitable for
this destination. Bind a composed result with PremultipliedAlpha=true in the
spatial material, which converts sampled color to straight alpha before lighting
and transparency. Ordinary source RSI/PNG textures use false. The cache restores
caller transform, shader, modulation and scissor state. Nested Clyde target
drawing also restores the outer target after exceptions and discards unfinished
inner draw commands.

## Qualification

`spatial_render_probe` captures original directional RSI art for two observers,
shared primitive residency and per-call uploads, alpha composition and perspective
rebinding, shadows, source invalidation and failed nested draw recovery. Unit tests
cover direction, steep views, pivot/physical size, folded timing, moving decals and
holes, signature sharing, pinned budgets and resource invalidation. These are
engine fixtures. They do not establish content asset coverage or gameplay readiness.

[Particles and screen effects](spatial-effects.md) provide bounded decoration and
post-processing; [XYZ audio](spatial-audio.md) provides the low-level source/listener
mechanism. E05/B03 view
and gameplay integration, B01 complete resource coverage and hardware performance
acceptance remain required by the plans.
