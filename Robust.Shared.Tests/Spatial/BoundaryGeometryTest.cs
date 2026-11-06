using System.Numerics;
using NUnit.Framework;
using Robust.Shared.GameObjects;
using Robust.Shared.Maths;
using Robust.Shared.Spatial;

namespace Robust.Shared.Tests.Spatial;

[TestFixture]
public sealed class BoundaryGeometryTest
{
    private static readonly BoundaryKey Key = new(new EntityUid(10), new LayerId(90), 0, 0, BoundaryAxis.Horizontal);
    private static readonly SurfaceId Surface = new(120);

    [Test]
    public void WindowRetainsJambsSillAndLintelWithOneSemanticSource()
    {
        var openings = new[] { new BoundaryOpening(0.25f, 0.75f, 1, 2) };
        var segment = new BoundarySegment(Key, Surface, 0.2f, 0.5f, 3, openings, "test-material", new EntityUid(11), 42);
        openings[0] = new BoundaryOpening(0, 1, 0, 3);
        var solids = segment.BuildSolidParts(new LayerDefinition(-4, "Deck"));
        Assert.That(solids, Has.Count.EqualTo(4));
        Assert.That(solids.Sum(s => s.Bounds.Volume), Is.EqualTo(0.5).Within(1e-6));
        Assert.That(solids.Any(s => s.Bounds.Contains(new Vector3(0.5f, 0, -2))), Is.False);
        foreach (var point in new[]
                 {
                     new Vector3(0.1f, 0, -2), new Vector3(0.9f, 0, -2),
                     new Vector3(0.5f, 0, -3), new Vector3(0.5f, 0, -1),
                 })
            Assert.That(solids.Any(s => s.Bounds.Contains(point)), Is.True);
        foreach (var solid in solids)
        {
            Assert.That(solid.Surface, Is.EqualTo(new SpatialSurfaceReference(Key.OwnerFrame, Surface)));
            Assert.That(solid.Entity, Is.EqualTo(new EntityUid(11)));
            Assert.That(solid.Boundary, Is.EqualTo(Key));
            Assert.That(solid.MaterialProfile, Is.EqualTo("test-material"));
            Assert.That(solid.Revision, Is.EqualTo(42));
        }
        Assert.Throws<NotSupportedException>(() => ((IList<BoundaryOpening>) segment.Openings).Clear());
    }

    [Test]
    public void OverlappingOpeningsAreSubtractedAsAUnionWithoutOverlappingSolids()
    {
        var holes = new[] { new BoundaryOpening(0.1f, 0.6f, 0.5f, 2), new BoundaryOpening(0.4f, 0.9f, 1, 2.5f) };
        var layer = new LayerDefinition(0, "Deck");
        var solids = new BoundarySegment(Key, Surface, 0.2f, 0, 3, holes).BuildSolidParts(layer);
        Assert.That(solids.Sum(s => s.Bounds.Volume), Is.EqualTo(0.34).Within(1e-6));
        for (var x = 0; x < 32; x++)
        for (var z = 0; z < 32; z++)
        {
            var point = new Vector3((x + 0.5f) / 32, 0, (z + 0.5f) * 3 / 32);
            var inVoid = holes.Any(o => point.X > o.Start && point.X < o.End && point.Z > o.Bottom && point.Z < o.Top);
            Assert.That(solids.Any(s => s.Bounds.Contains(point)), Is.EqualTo(!inVoid), $"Incorrect geometry at {point}");
        }
        for (var first = 0; first < solids.Count; first++)
        for (var second = first + 1; second < solids.Count; second++)
        {
            var a = solids[first].Bounds;
            var b = solids[second].Bounds;
            var overlap = Vector3.Min(a.Max, b.Max) - Vector3.Max(a.Min, b.Min);
            Assert.That(overlap.X > 0 && overlap.Y > 0 && overlap.Z > 0, Is.False);
        }
        var duplicateReordered = new BoundarySegment(Key, Surface, 0.2f, 0, 3,
            new[] { holes[1], holes[0], holes[1] }).BuildSolidParts(layer);
        Assert.That(duplicateReordered.Select(s => s.Bounds), Is.EqualTo(solids.Select(s => s.Bounds)));
    }

    [Test]
    public void DoorAndFullHeightGapDoNotCreateAnInvisibleThreshold()
    {
        var layer = new LayerDefinition(7.25f, "Uneven deck", 2);
        var key = new BoundaryKey(Key.OwnerFrame, Key.LayerId, -3, -2, BoundaryAxis.Vertical);
        var door = new BoundarySegment(key, Surface, 0.15f, 0, 3,
            new[] { new BoundaryOpening(0.25f, 1.75f, 0, 2.5f) });
        var solids = door.BuildSolidParts(layer);
        Assert.That(solids.Any(s => s.Bounds.Contains(new Vector3(-6, -3, 7.2501f))), Is.False);
        Assert.That(solids.Any(s => s.Bounds.Contains(new Vector3(-6, -3, 10))), Is.True);
        var empty = new BoundarySegment(key, Surface, 0.15f, 0, 3,
            new[] { new BoundaryOpening(0, 2, 0, 3) }).BuildSolidParts(layer);
        Assert.That(empty, Is.Empty);
    }

    [Test]
    public void InvalidOrUnrepresentableGeometryIsRejected()
    {
        Assert.Throws<ArgumentException>(() => new BoundaryOpening(0, 1, 1, 1));
        Assert.Throws<ArgumentException>(() => new BoundaryOpening(float.NaN, 1, 0, 1));
        Assert.Throws<ArgumentException>(() => new BoundarySegment(Key, Surface, 0, 0, 3));
        Assert.Throws<ArgumentException>(() => new BoundarySegment(Key, Surface, 0.2f, 0, 3,
            new[] { new BoundaryOpening(0, 1, 0, 4) }));
        var outside = new BoundarySegment(Key, Surface, 0.2f, 0, 3, new[] { new BoundaryOpening(0, 2, 0, 1) });
        Assert.Throws<ArgumentException>(() => outside.BuildSolidParts(new LayerDefinition(0, "Deck")));
        var distant = new BoundarySegment(new BoundaryKey(Key.OwnerFrame, Key.LayerId, int.MaxValue - 1, 0, BoundaryAxis.Horizontal),
            Surface, 0.2f, 0, 3);
        Assert.Throws<ArgumentException>(() => distant.BuildSolidParts(new LayerDefinition(0, "Deck")));
        Assert.Throws<ArgumentException>(() => new Box3(Vector3.Zero, new Vector3(1, 1, float.PositiveInfinity)));
        Assert.Throws<ArgumentException>(() => new Box3(new Vector3(-float.MaxValue), new Vector3(float.MaxValue)));
        Assert.That(default(Box3).IsValid, Is.False);
    }
}
