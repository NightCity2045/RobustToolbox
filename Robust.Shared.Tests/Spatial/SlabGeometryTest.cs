using System.Numerics;
using NUnit.Framework;
using Robust.Shared.GameObjects;
using Robust.Shared.Maths;
using Robust.Shared.Spatial;

namespace Robust.Shared.Tests.Spatial;

[TestFixture]
public sealed class SlabGeometryTest
{
    private static readonly SpatialSurfaceReference Surface = new(new EntityUid(1), new SurfaceId(2));
    private static readonly LayerId Layer = new(99);

    [Test]
    public void ShaftIsEmptyThroughEntireStructuralThickness()
    {
        var holes = new[] { new Box2(-0.5f, -0.5f, 0.5f, 0.5f) };
        var slab = new SlabSurface(Surface, Layer, new Box2(-2, -2, 2, 2), 0, 0.2f, holes, "structural", stateRevision: 7);
        holes[0] = slab.Footprint;
        var parts = slab.BuildSolidParts(new LayerDefinition(3.6f, "Deck"));
        Assert.That(parts.Sum(p => p.Bounds.Volume), Is.EqualTo(3).Within(1e-5));
        foreach (var z in new[] { 3.4001f, 3.5f, 3.5999f })
        {
            Assert.That(parts.Any(p => p.Bounds.Contains(new Vector3(0, 0, z))), Is.False);
            Assert.That(parts.Any(p => p.Bounds.Contains(new Vector3(1, 0, z))), Is.True);
        }
        foreach (var part in parts)
        {
            Assert.That(part.Surface, Is.EqualTo(Surface));
            Assert.That(part.Entity, Is.Null);
            Assert.That(part.Boundary, Is.Null);
            Assert.That(part.Revision, Is.EqualTo(7));
        }
        Assert.Throws<NotSupportedException>(() => ((IList<Box2>) slab.Openings).Clear());
    }

    [Test]
    public void BridgeAboveLowerSurfaceHasIndependentVolumeAndIdentity()
    {
        var bridge = new SlabSurface(Surface, Layer, new Box2(-4, -0.5f, 4, 0.5f), 0, 0.3f);
        var lowerRef = new SpatialSurfaceReference(Surface.OwnerFrame, new SurfaceId(3));
        var lower = new SlabSurface(lowerRef, new LayerId(4), new Box2(-4, -4, 4, 4), 0, 0.2f);
        var bridgeParts = bridge.BuildSolidParts(new LayerDefinition(2.35f, "Bridge"));
        var lowerParts = lower.BuildSolidParts(new LayerDefinition(-1.1f, "Ground"));
        Assert.That(bridgeParts, Has.Count.EqualTo(1));
        Assert.That(lowerParts, Has.Count.EqualTo(1));
        Assert.That(bridgeParts[0].Bounds.Intersects(lowerParts[0].Bounds), Is.False);
        Assert.That(bridgeParts[0].Surface, Is.Not.EqualTo(lowerParts[0].Surface));
        Assert.That(bridgeParts[0].Bounds.Contains(new Vector3(0, 2, 2.2f)), Is.False);
        Assert.That(lowerParts[0].Bounds.Contains(new Vector3(0, 2, -1.2f)), Is.True);
    }

    [Test]
    public void FullyOpenSlabHasNoSolidAndInvalidFootprintsAreRejected()
    {
        var footprint = new Box2(-2, -1, 2, 1);
        var slab = new SlabSurface(Surface, Layer, footprint, 0, 0.2f, new[] { footprint });
        Assert.That(slab.BuildSolidParts(new LayerDefinition(0, "Deck")), Is.Empty);
        Assert.Throws<ArgumentException>(() => new SlabSurface(Surface, Layer, footprint, 0, 0.2f,
            new[] { new Box2(-3, -1, 2, 1) }));
        Assert.Throws<ArgumentException>(() => new SlabSurface(Surface, Layer, footprint, 0, -0.2f));
        Assert.Throws<ArgumentException>(() => new SlabSurface(Surface, Layer, footprint, float.PositiveInfinity, 0.2f));
        Assert.Throws<ArgumentException>(() => new SlabSurface(Surface, Layer, new Box2(0, 0, 0, 1), 0, 0.2f));
    }
}
