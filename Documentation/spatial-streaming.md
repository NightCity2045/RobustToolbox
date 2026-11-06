# Authorized geometry transport

Geometry uses a reliable ordered channel separate from ordinary entity messages.
The server builds a selection after PVS generation from the same authorized XYZ
views and surface/source policy. Normal entity states still own transforms,
parents, world identity, body definitions and velocities. Geometry messages never
instantiate entities or supply another pose. A denied surface is excluded together
with its decorative tiles and source references. Denied collision also prevents
the server from certifying that volume as complete.

Each frame's normal component state carries its geometry component lifetime GUID
and revision. Removing/recreating the component cannot reuse an old wire lifetime.
The semantic payload carries layers, boundaries/openings, slabs/openings, sparse
decorative tiles and a bounded NetEntity reference table. Payload hashes cover the
exact bytes. Decoding resolves the reference table before constructing geometry;
unknown parents/sources defer installation without creating placeholders.

A manifest identifies its sequence, acknowledged base, authoritative tick, frame
descriptions and complete authorized volumes. The client atomically installs all
frames only after receiving every fragment, reaching the authoritative tick,
resolving the dependencies and matching the component lifetime/revision stamps.
Unchanged frames in a delta reuse immutable payloads/snapshots. Full baselines
consume every fragment even when identical bytes were previously cached, so the
sender cannot be stranded waiting for progress acknowledgements.

The transport splits payloads into 16 KiB fragments. Each session has at most
16 outstanding fragments (256 KiB); progress acknowledgements release that window.
`net.spatial_geometry_bytes` defaults to 65536 bytes per server publication and is
clamped to 64 KiB–4 MiB. The final acknowledgement advances the baseline only after
all fragments were sent and acknowledged. Changed geometry, revoked source access
or removed viewers cancel an incomplete transfer. Range churn may finish an
already authorized transfer before starting the next selection. Client inactivity
over five seconds requests a new baseline; server inactivity over ten seconds
abandons the old transfer. Repeated baseline requests are rate limited.

The receiver bounds a manifest to 512 frames, 32 views and 32 MiB of payloads,
each frame to 8 MiB, each collection to 16384 records, UTF-8 text to 4096 bytes,
and decoded solids to 262144. Opening partition work is also bounded before
geometry expansion. Invalid counts, references, UTF-8, hashes, trailing data,
conflicting duplicates and missing bases cannot install partial geometry.
These are codec/resource limits, not measured performance qualification. Large
selections still need chunk scheduling within these limits before full G7 closure.

`Robust.Client.Spatial.SpatialGeometryStreamingSystem` exposes read-only snapshots,
their identity/currentness and `IsReady(world, bounds)`. False means unknown
collision data. It must never be interpreted as an empty scene. Pending manifests
and newly received geometry stamps invalidate coverage; an older in-flight
manifest cannot clear a newer frame invalidation. A newer manifest may explicitly
certify that a removed/out-of-range frame no longer contributes to coverage.
Reparenting across worlds immediately invalidates cached frames whose owners or
semantic references no longer resolve in their recorded world, even with unchanged
geometry stamps. A new-world manifest must establish coverage again.
These APIs do not themselves implement the prediction barrier or client physics.

Network registration belongs to the persistent entity manager. Message callbacks
look up the current streaming system, whose lifetime ends on client disconnect.
The old replica and pending buffers are cleared on disconnect, shutdown and entity
flush; a subsequent session establishes a new baseline.

Tests cover semantic reference remapping and visibility filtering, every byte
truncation of representative payloads/messages, allocation/partition bounds,
out-of-order and duplicate fragments, missing dependencies/ticks/bases, recreated
components and invalidation during transfer. Client/server integration exercises
a payload larger than the progress window, full reset with cached bytes, a
two-frame transaction, hidden geometry and reconnect after geometry removal,
with both ordinary PVS and the explicit server PVS-off mode.

Remaining E07 work includes chunk scheduling for oversized selections, coherent
collision consumption during state/interest churn, local prediction/reconciliation,
prediction integration with the available pose/geometry history, replay capture and loss/jitter qualification. Passing these
transport tests does not close G7.
