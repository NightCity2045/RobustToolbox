using System;
using System.Collections.Generic;
using System.Linq;
using Robust.Shared.Maths;

namespace Robust.Shared.Spatial;

/// <summary>Partition a rectangle minus a union of rectangular voids into disjoint rectangles.</summary>
internal static class PlanarRegionPartition
{
    internal static void ValidateRectangle(Box2 rectangle)
    {
        if (!float.IsFinite(rectangle.Left) || !float.IsFinite(rectangle.Right)
            || !float.IsFinite(rectangle.Bottom) || !float.IsFinite(rectangle.Top)
            || rectangle.Right <= rectangle.Left || rectangle.Top <= rectangle.Bottom
            || !float.IsFinite(rectangle.Width) || !float.IsFinite(rectangle.Height))
            throw new ArgumentException("A region must be finite with positive dimensions.");
    }

    internal static void ValidateOpening(Box2 bounds, Box2 opening)
    {
        ValidateRectangle(opening);
        if (opening.Left < bounds.Left || opening.Right > bounds.Right
            || opening.Bottom < bounds.Bottom || opening.Top > bounds.Top)
            throw new ArgumentException("An opening extends beyond its structural region.");
    }

    internal static List<Box2> Subtract(Box2 bounds, IEnumerable<Box2> openings)
    {
        ValidateRectangle(bounds);
        var holes = openings.ToArray();
        var cuts = new SortedSet<float> { bounds.Left, bounds.Right };
        foreach (var opening in holes)
        {
            ValidateOpening(bounds, opening);
            cuts.Add(opening.Left);
            cuts.Add(opening.Right);
        }

        var ordered = cuts.ToArray();
        var rectangles = new List<Box2>();
        var previous = new Dictionary<(float Bottom, float Top), int>();
        for (var column = 0; column < ordered.Length - 1; column++)
        {
            var start = ordered[column];
            var end = ordered[column + 1];
            var covered = holes.Where(o => o.Left <= start && o.Right >= end)
                .OrderBy(o => o.Bottom).ThenBy(o => o.Top);
            var current = new Dictionary<(float Bottom, float Top), int>();
            var bottom = bounds.Bottom;
            foreach (var opening in covered)
            {
                if (opening.Bottom > bottom)
                    AddRectangle(bottom, opening.Bottom);
                bottom = Math.Max(bottom, opening.Top);
            }
            if (bottom < bounds.Top)
                AddRectangle(bottom, bounds.Top);
            previous = current;

            void AddRectangle(float low, float high)
            {
                var span = (low, high);
                if (previous.TryGetValue(span, out var index))
                {
                    var extended = rectangles[index];
                    extended.Right = end;
                    rectangles[index] = extended;
                }
                else
                {
                    index = rectangles.Count;
                    rectangles.Add(new Box2(start, low, end, high));
                }
                current.Add(span, index);
            }
        }
        return rectangles;
    }
}
