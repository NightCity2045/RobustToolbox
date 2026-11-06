using Robust.Shared.GameObjects;
using Robust.Shared.Maths;

namespace Robust.Shared.Spatial;

/// <summary>
/// A derived solid in its source frame. Physics, picking and rendering consume the
/// same geometry and retain its semantic source even when batching shapes/meshes.
/// </summary>
public readonly record struct SpatialSolidPart(
    SpatialSurfaceReference Surface,
    EntityUid? Entity,
    Box3 Bounds,
    string MaterialProfile,
    ulong Revision,
    BoundaryKey? Boundary = null);
