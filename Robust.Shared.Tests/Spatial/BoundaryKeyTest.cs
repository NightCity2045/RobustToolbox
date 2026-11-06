using NUnit.Framework;
using Robust.Shared.GameObjects;
using Robust.Shared.Maths;
using Robust.Shared.Serialization;
using Robust.Shared.Spatial;

namespace Robust.Shared.Tests.Spatial;

[TestFixture]
public sealed class BoundaryKeyTest
{
    private static readonly EntityUid Frame = new(123);
    private static readonly LayerId Layer = new(71);

    [TestCase(-12, -25)]
    [TestCase(-1, -1)]
    [TestCase(0, 0)]
    [TestCase(35, 61)]
    public void NeighboursShareOneBoundaryAcrossEveryPrefabSymmetry(int x, int y)
    {
        var cell = new Vector2i(x, y);
        var east = BoundaryKey.FromCell(Frame, Layer, cell, CellBoundary.East);
        var west = BoundaryKey.FromCell(Frame, Layer, new Vector2i(x + 1, y), CellBoundary.West);
        var north = BoundaryKey.FromCell(Frame, Layer, cell, CellBoundary.North);
        var south = BoundaryKey.FromCell(Frame, Layer, new Vector2i(x, y + 1), CellBoundary.South);
        Assert.That(east, Is.EqualTo(west));
        Assert.That(north, Is.EqualTo(south));
        foreach (var mirror in new[] { false, true })
        for (var turn = 0; turn < 4; turn++)
        {
            var offset = new Vector2i(-17, 9);
            Assert.That(east.Transform(Frame, Layer, turn, mirror, offset),
                Is.EqualTo(west.Transform(Frame, Layer, turn, mirror, offset)));
            Assert.That(north.Transform(Frame, Layer, turn, mirror, offset),
                Is.EqualTo(south.Transform(Frame, Layer, turn, mirror, offset)));
        }
    }

    [Test]
    public void RotationAndReflectionNormalizeAgainstKnownGridEdges()
    {
        var edge = new BoundaryKey(Frame, Layer, -3, 2, BoundaryAxis.Horizontal);
        Assert.That(edge.Transform(Frame, Layer, 1, false, Vector2i.Zero),
            Is.EqualTo(new BoundaryKey(Frame, Layer, -2, -3, BoundaryAxis.Vertical)));
        Assert.That(edge.Transform(Frame, Layer, 0, true, Vector2i.Zero),
            Is.EqualTo(new BoundaryKey(Frame, Layer, 2, 2, BoundaryAxis.Horizontal)));
        Assert.That(edge.Transform(Frame, Layer, 1, true, new Vector2i(10, -4)),
            Is.EqualTo(new BoundaryKey(Frame, Layer, 8, -2, BoundaryAxis.Vertical)));
        var rotated = edge;
        for (var i = 0; i < 4; i++)
            rotated = rotated.Transform(Frame, Layer, 1, false, Vector2i.Zero);
        Assert.That(rotated, Is.EqualTo(edge));
        Assert.That(edge.Transform(Frame, Layer, 0, true, Vector2i.Zero)
            .Transform(Frame, Layer, 0, true, Vector2i.Zero), Is.EqualTo(edge));
        var (start, end) = edge.Endpoints();
        Assert.That(BoundaryKey.FromEndpoints(Frame, Layer, end, start), Is.EqualTo(edge));
    }

    [Test]
    public void AddressIncludesFrameAndLogicalLayerWithoutDerivingElevation()
    {
        var edge = new BoundaryKey(Frame, Layer, 0, 0, BoundaryAxis.Horizontal);
        var otherFrame = edge.Transform(new EntityUid(456), Layer, 0, false, Vector2i.Zero);
        var otherLayer = edge.Transform(Frame, new LayerId(999), 0, false, Vector2i.Zero);
        Assert.That(new HashSet<BoundaryKey> { edge, otherFrame, otherLayer }, Has.Count.EqualTo(3));
        // At a T-junction, reversing individual edges deduplicates those edges,
        // while the three distinct incident construction segments remain distinct.
        var edges = new HashSet<BoundaryKey>();
        foreach (var endpoint in new[] { Vector2i.Left, Vector2i.Right, Vector2i.Up })
        {
            edges.Add(BoundaryKey.FromEndpoints(Frame, Layer, Vector2i.Zero, endpoint));
            edges.Add(BoundaryKey.FromEndpoints(Frame, Layer, endpoint, Vector2i.Zero));
        }
        Assert.That(edges, Has.Count.EqualTo(3));
    }

    [Test]
    public void InvalidEdgesAndUnrepresentableCoordinatesAreRejected()
    {
        Assert.Throws<ArgumentException>(() => new BoundaryKey(EntityUid.Invalid, Layer, 0, 0, BoundaryAxis.Horizontal));
        Assert.Throws<ArgumentOutOfRangeException>(() => new BoundaryKey(Frame, Layer, 0, 0, (BoundaryAxis) 99));
        Assert.Throws<ArgumentOutOfRangeException>(() => new BoundaryKey(Frame, Layer, int.MaxValue, 0, BoundaryAxis.Horizontal));
        Assert.Throws<ArgumentException>(() => BoundaryKey.FromEndpoints(Frame, Layer, Vector2i.Zero, Vector2i.One));
        Assert.Throws<ArgumentException>(() => BoundaryKey.FromEndpoints(Frame, Layer, Vector2i.Zero, Vector2i.Zero));
        Assert.Throws<ArgumentException>(() => BoundaryKey.FromEndpoints(Frame, Layer, new Vector2i(int.MinValue, 0), new Vector2i(int.MaxValue, 0)));
        Assert.Throws<OverflowException>(() => BoundaryKey.FromCell(Frame, Layer, new Vector2i(int.MaxValue, 0), CellBoundary.East));
        var edge = new BoundaryKey(Frame, Layer, int.MinValue, 0, BoundaryAxis.Horizontal);
        Assert.Throws<OverflowException>(() => edge.Transform(Frame, Layer, 0, true, Vector2i.Zero));
        Assert.That(edge.Transform(Frame, Layer, 0, true, new Vector2i(-1, 0)),
            Is.EqualTo(new BoundaryKey(Frame, Layer, int.MaxValue - 1, 0, BoundaryAxis.Horizontal)));
    }

    [Test]
    public void LayerIdUsesInvariantStableScalar()
    {
        ISelfSerialize id = new LayerId(uint.MaxValue);
        Assert.That(id.Serialize(), Is.EqualTo("4294967295"));
        id.Deserialize("71");
        Assert.That((LayerId) id, Is.EqualTo(Layer));
        Assert.Throws<FormatException>(() => id.Deserialize("-1"));
        Assert.Throws<FormatException>(() => id.Deserialize("1.0"));
        Assert.Throws<OverflowException>(() => id.Deserialize("4294967296"));
        Assert.That((LayerId) id, Is.EqualTo(Layer));
    }
}
