# Spatial particles and screen effects

Own a `SpatialParticlePool` per client world on the client thread. Its count and
maximum lifetime are explicit limits. `TryEmit` rejects excess decoration without
evicting existing emissions. `Advance` ages particles; `CreateVisuals` evaluates
analytical XYZ motion, size and color for each observer. Rendering frequency and
observer direction do not change their positions. These are visual records, not
projectiles, entities or collision bodies.

An optional frame attaches motion to that frame's current full pose. Supply only
authorized frame poses; a missing frame expires its particles. Call `RemoveOwner`
when the source is deleted or authorization is revoked, and clear/dispose the pool
when leaving the world. Content supplies emission events, profiles, materials,
colors and lifetime. Reserve mandatory scene information before allocating the
decorative budget. No quality setting changes physical or gameplay state.

Particle records use the ordinary spatial material pipeline, including opaque
depth, cutout and weighted transparency. Per-instance linear RGBA modulation
allows color/fade without allocating a material for every particle. Billboard
orientation remains observer-specific; GroundSprite/SurfaceQuad can carry explicit
rotating presentation poses. Pool motion never writes canonical transforms.

## Screen passes

Own one `SpatialScreenEffects` per view. Apply an ordered list of content-owned
`ShaderInstance`s to the completed scene before composing ordinary UI. Each shader
reads the previous pass through `TEXTURE`/`UV` (`zTexture(UV)`), not the implicit
screen texture. Use explicit shader blending suitable for a complete screen image;
the passes clear to opaque black and have no depth or stencil attachment.
The chain restores the caller's transform, shader, modulation, target and scissor.

Two RGBA8 sRGB targets are reused and replaced together on resize. ResidentBytes
reports their combined storage; MaximumPixels bounds each target and MaximumPasses
bounds the chain. No passes returns the input without allocation. Input must not be
either of the chain's own output textures, including atlas aliases. Reusing an old
output as input would create feedback in later passes and is rejected.

An excessive budget returns BudgetExceeded with no texture and clears previously
exposed owned output to opaque black. Invalid shaders or draw failures also clear
owned output and throw. The caller must handle failure without substituting an
unfiltered scene: blindness and other vision effects are content policy, not
optional quality. Do not retain returned textures across resize or disposal.
CopyPixelsToMemory reads the most recent successfully completed nonempty chain.
Shader ownership and visibility policy remain with content.

## Qualification

Particle tests cover XYZ motion, time partitioning, observer independence, moving
frames, expiry, owner revocation, finite values and budgets. The real GL
`spatial_render_probe` checks modulation, occlusion, order-independent transparency,
ordered screen passes, target reuse/resize, atlas feedback rejection and opaque
black output on budget failure. These engine fixtures do not establish integrated
gameplay, complete content effects or hardware performance acceptance.
