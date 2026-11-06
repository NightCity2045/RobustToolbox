using System.Numerics;
using NUnit.Framework;
using Robust.Shared.GameObjects;
using Robust.Shared.Maths;
using Robust.Shared.Spatial;

namespace Robust.Shared.Tests.Spatial;

[TestFixture]
public sealed class SpatialSurfaceBuilderTest
{
    private static readonly EntityUid Frame = new(1);
    private static readonly LayerId Layer = new(0);

    [Test]
    public void CornersAndTJunctionsHaveOnlyExteriorFacesAndKeepSemanticSources()
    {
        foreach (var stem in new[] { new Box2(0.5f, -1, 1.5f, 0), new Box2(0, -1, 1, 1) })
        {
            var slabs = new[] { Slab(1, new Box2(0, 0, 2, 1)), Slab(2, stem) };
            var snapshot = Snapshot(slabs: slabs);
            var faces = SpatialSurfaceBuilder.Build(snapshot);
            Assert.That(faces.Sum(f => f.Area), Is.EqualTo(14).Within(1e-6));
            Assert.That(faces.Select(f => f.Source.Surface).Distinct().Count(), Is.EqualTo(2));
            Assert.That(snapshot.Slabs, Has.Count.EqualTo(2));
            AssertExterior(snapshot, faces);
        }
    }

    [Test]
    public void CoincidentSurfacesProduceOneSkinIndependentOfDefinitionOrder()
    {
        var first = Slab(2, new Box2(0, 0, 1, 1));
        var second = Slab(1, new Box2(0, 0, 1, 1));
        foreach (var slabs in new[] { new[] { first, second }, new[] { second, first } })
        {
            var snapshot = Snapshot(slabs: slabs);
            var faces = SpatialSurfaceBuilder.Build(snapshot);
            Assert.That(faces, Has.Count.EqualTo(6));
            Assert.That(faces.Sum(f => f.Area), Is.EqualTo(6));
            Assert.That(faces.All(f => f.Source.Surface.Surface == new SurfaceId(1)), Is.True);
            Assert.That(snapshot.ContainsSurface(new SurfaceId(2)), Is.True);
            AssertExterior(snapshot, faces);
        }
    }

    [Test]
    public void WindowAndShaftHaveVisibleRevealFacesWithoutClosingTheirVoids()
    {
        var boundary = new BoundarySegment(new BoundaryKey(Frame, Layer, 0, 0, BoundaryAxis.Horizontal),
            new SurfaceId(1), 0.2f, 0, 3, new[] { new BoundaryOpening(0.25f, 0.75f, 1, 2) },
            "window-frame", Frame, 7);
        var wall = Snapshot(boundaries: new[] { boundary });
        var windowFaces = SpatialSurfaceBuilder.Build(wall);
        Assert.That(windowFaces.Sum(f => f.Area), Is.EqualTo(7.2).Within(1e-5));
        Assert.That(windowFaces.All(f => f.Source.Boundary == boundary.Key
            && f.Source.Entity == Frame && f.Source.MaterialProfile == "window-frame"
            && f.Source.Revision == 7), Is.True);
        AssertExterior(wall, windowFaces);

        var slab = new SlabSurface(new SpatialSurfaceReference(Frame, new SurfaceId(2)), Layer,
            new Box2(-1, -1, 1, 1), 0, 0.2f, new[] { new Box2(-0.5f, -0.5f, 0.5f, 0.5f) });
        var shaft = Snapshot(slabs: new[] { slab });
        var shaftFaces = SpatialSurfaceBuilder.Build(shaft);
        // Top/bottom: 2*(4-1), outer perimeter: 8*.2, shaft perimeter: 4*.2.
        Assert.That(shaftFaces.Sum(f => f.Area), Is.EqualTo(8.4).Within(1e-5));
        AssertExterior(shaft, shaftFaces);
    }

    private static SlabSurface Slab(uint id, Box2 footprint)
        => new(new SpatialSurfaceReference(Frame, new SurfaceId(id)), Layer, footprint, 1, 1);

    [Test]
    public void SparseMultiLayerRegionsRemainIndependentWhenBuildingExterior()
    {
        var layers = new Dictionary<LayerId, LayerDefinition>();
        var slabs = new List<SlabSurface>();
        for (uint layer = 0; layer < 8; layer++)
        {
            var id = new LayerId(layer);
            layers[id] = new LayerDefinition(layer * 7.25f, "Deck");
            for (var x = -8; x < 8; x++)
            for (var y = -8; y < 8; y++)
                slabs.Add(new SlabSurface(new SpatialSurfaceReference(Frame, new SurfaceId((uint) slabs.Count)),
                    id, new Box2(x * 17, y * 17, x * 17 + 1, y * 17 + 1), 1, 1));
        }
        var snapshot = new SpatialGeometrySnapshot(Frame, 0, layers, Array.Empty<BoundarySegment>(), slabs,
            Array.Empty<SpatialTileChunk>());
        var faces = SpatialSurfaceBuilder.Build(snapshot);
        Assert.That(faces, Has.Count.EqualTo(slabs.Count * 6));
        Assert.That(faces.Sum(f => f.Area), Is.EqualTo(slabs.Count * 6));
        Assert.That(faces.Select(f => f.Source.Surface).Distinct().Count(), Is.EqualTo(slabs.Count));
    }

    [Test]
    public void VolumeIndexMatchesBruteForceAcrossTouchingAndSeparatedRegions()
    {
        var random = new System.Random(295);
        var solids = new List<SpatialSolidPart>();
        for (var i = 0; i < 512; i++)
        {
            var min = new Vector3(random.Next(-20, 20), random.Next(-20, 20), random.Next(-20, 20));
            var size = new Vector3(random.Next(1, 6), random.Next(1, 6), random.Next(1, 6));
            solids.Add(new SpatialSolidPart(new SpatialSurfaceReference(Frame, new SurfaceId((uint) i)), null,
                new Box3(min, min + size), "", 0));
        }
        var index = new SpatialSolidIndex(solids);
        var matches = new List<int>();
        var stack = new Stack<int>();
        foreach (var solid in solids)
        {
            index.FindIntersections(solid.Bounds, matches, stack);
            var expected = Enumerable.Range(0, solids.Count).Where(i => solids[i].Bounds.Intersects(solid.Bounds));
            Assert.That(matches, Is.EquivalentTo(expected));
        }
    }

    private static SpatialGeometrySnapshot Snapshot(BoundarySegment[]? boundaries = null, SlabSurface[]? slabs = null)
        => new(Frame, 7, new Dictionary<LayerId, LayerDefinition> { [Layer] = new(0, "Deck") },
            boundaries ?? Array.Empty<BoundarySegment>(), slabs ?? Array.Empty<SlabSurface>(), Array.Empty<SpatialTileChunk>());

    private static void AssertExterior(SpatialGeometrySnapshot snapshot, IReadOnlyList<SpatialSurfacePatch> faces)
    {
        foreach (var face in faces)
        {
            var center = (face.GetVertex(0) + face.GetVertex(2)) / 2;
            Assert.That(snapshot.SolidParts.Any(s => s.Bounds.Contains(center - face.Normal * 0.0001f)), Is.True);
            Assert.That(snapshot.SolidParts.Any(s => s.Bounds.Contains(center + face.Normal * 0.0001f)), Is.False,
                $"An internal face survived at {center}, normal {face.Normal}.");
            var winding = Vector3.Cross(face.GetVertex(1) - face.GetVertex(0), face.GetVertex(2) - face.GetVertex(0));
            Assert.That(Vector3.Dot(winding, face.Normal), Is.GreaterThan(0));
        }
        for (var i = 0; i < faces.Count; i++)
        for (var j = i + 1; j < faces.Count; j++)
        {
            var a = faces[i];
            var b = faces[j];
            if (a.Direction != b.Direction || a.Plane != b.Plane)
                continue;
            var width = Math.Min(a.Region.Right, b.Region.Right) - Math.Max(a.Region.Left, b.Region.Left);
            var height = Math.Min(a.Region.Top, b.Region.Top) - Math.Max(a.Region.Bottom, b.Region.Bottom);
            Assert.That(width <= 0 || height <= 0, Is.True, "Coplanar exterior faces overlap.");
        }
    }
}
