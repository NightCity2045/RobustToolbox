# Canonical spatial geometry (E02, in progress)

Geometry belongs to the existing transform frame. `SpatialFrameComponent` owns
local layer definitions; `SpatialGeometryComponent` stores schema version,
revision, boundary segments and structural slabs. No independent XYZ entity
positions or physics handles are stored here.

`BoundaryKey` addresses a unit authoring-grid edge by owner frame, local layer,
integer X/Y and Horizontal/Vertical axis. The lesser endpoint extends toward +X
or +Y. Adjacent cells and reversed endpoints identify the same construction.
Grid transforms mirror X, rotate counterclockwise in quarter turns, then translate
and normalize. Checked wider arithmetic supports negative coordinates without
wrapping. Arbitrary world rotations remain in the frame transform.

`LayerId` and `SurfaceId` are stable local identifiers, scoped by their owner frame.
They are not elevations or global entity IDs. The map serializer remaps owner and
optional source entity references independently for each prefab instance. Runtime
references containing EntityUid deliberately lack NetSerializable; wire records
must use NetEntity. A surface ID survives decomposition into multiple solid parts.

`BoundarySegment` supplies thickness, base elevation, height, material identifier,
revision and real rectangular openings. Subtracting their union preserves jambs,
sills and lintels; overlapping openings do not duplicate voids or solid parts.
`SlabSurface` supplies a footprint, top elevation, downward thickness and through
openings. One slab supplies both floor and ceiling collision. A bridge, a lower
floor and an open shaft can occupy the same XY region at independent elevations.
Decorative tiles are separate from structural slabs.

Both forms produce frame-local `SpatialSolidPart` boxes with semantic surface,
source entity, material and revision references. This CPU representation is usable
without a GPU or Content dependency. Real BEPU ray/sphere-sweep qualification tests
consume these same boxes, including a rotated frame, a window, slab top/underside
and a clear shaft. This is not yet an ECS physics backend or render mesh builder;
corner unions and internal-face removal remain work for geometry consumers.

`SharedGeometrySystem.ConfigureInitialGeometry` validates a whole candidate before
replacing initial canonical data. It rejects duplicate edges, duplicate surface
IDs across walls and slabs, foreign frames, missing layers and inconsistent
revisions. Live edits cannot use this initialization API. `CaptureSnapshot` copies
semantic definitions into read-only collections and derives solid parts once for
consumer jobs. A captured snapshot remains independent of later initialization
changes and deletion. Consumers must separately validate revisions before installing
job results; the runtime transaction mechanism is not implemented yet.

Map saves contain semantic definitions and revisions, never derived boxes, meshes
or backend indices. Integration tests save, restore and explicitly clone a world,
check entity/surface remapping, and reject malformed geometry. Geometry is not
replicated as an unbounded ordinary component state: chunk streaming remains E07.

E02/G2 remains open. Sparse tile chunks, complete corner/mesh topology, runtime
transactions and the query, physics, graph and renderer integrations still need
to be completed and qualified together with Content.
