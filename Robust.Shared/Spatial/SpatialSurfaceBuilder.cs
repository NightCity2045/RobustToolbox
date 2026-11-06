using System;
using System.Collections.Generic;
using System.Linq;
using Robust.Shared.Maths;

namespace Robust.Shared.Spatial;

/// <summary>
/// Builds the exterior of the canonical axis-aligned solids in a single frame.
/// Internal/coplanar faces are removed without merging away logical surfaces.
/// Run on a detached geometry snapshot; revision acceptance belongs to the caller.
/// </summary>
public static class SpatialSurfaceBuilder
{
    public static IReadOnlyList<SpatialSurfacePatch> Build(SpatialGeometrySnapshot snapshot)
    {
        ArgumentNullException.ThrowIfNull(snapshot);
        var solids = snapshot.SolidParts;
        var sorted = Enumerable.Range(0, solids.Count)
            .OrderBy(i => solids[i].Surface.Surface.Value).ThenBy(i => i).ToArray();
        var volumes = new SpatialSolidIndex(solids);
        var neighbors = new List<int>();
        var pending = new Stack<int>();
        var patches = new List<SpatialSurfacePatch>();
        foreach (var index in sorted)
        {
            var source = solids[index];
            var bounds = source.Bounds;
            volumes.FindIntersections(bounds, neighbors, pending);

            for (var face = 0; face < 6; face++)
            {
                var axis = face / 2;
                var positive = face % 2 != 0;
                var plane = Coordinate(positive ? bounds.Max : bounds.Min, axis);
                var region = Project(bounds, axis);
                var holes = new List<Box2>();
                foreach (var neighbor in neighbors)
                {
                    if (neighbor == index)
                        continue;
                    var other = solids[neighbor];
                    var min = Coordinate(other.Bounds.Min, axis);
                    var max = Coordinate(other.Bounds.Max, axis);
                    if (plane < min || plane > max)
                        continue;
                    // At an equal outward face, choose one stable semantic source.
                    // At the opposite face or inside another solid, this is internal.
                    var sameFace = positive ? plane == max : plane == min;
                    if (sameFace && (other.Surface.Surface.Value > source.Surface.Surface.Value
                        || other.Surface.Surface == source.Surface.Surface && neighbor > index))
                        continue;
                    var projected = Project(other.Bounds, axis);
                    var clip = new Box2(Math.Max(region.Left, projected.Left), Math.Max(region.Bottom, projected.Bottom),
                        Math.Min(region.Right, projected.Right), Math.Min(region.Top, projected.Top));
                    if (clip.Right > clip.Left && clip.Top > clip.Bottom)
                        holes.Add(clip);
                }
                foreach (var exterior in PlanarRegionPartition.Subtract(region, holes))
                    patches.Add(new SpatialSurfacePatch(source, (SpatialFaceDirection) face, plane, exterior));
            }
        }
        return patches.AsReadOnly();
    }

    private static float Coordinate(System.Numerics.Vector3 point, int axis)
        => axis switch { 0 => point.X, 1 => point.Y, _ => point.Z };

    private static Box2 Project(Box3 box, int axis) => axis switch
    {
        0 => new Box2(box.Min.Y, box.Min.Z, box.Max.Y, box.Max.Z),
        1 => new Box2(box.Min.X, box.Min.Z, box.Max.X, box.Max.Z),
        _ => new Box2(box.Min.X, box.Min.Y, box.Max.X, box.Max.Y)
    };
}
