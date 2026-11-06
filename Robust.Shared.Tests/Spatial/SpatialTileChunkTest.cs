using System;
using System.Collections.Generic;
using NUnit.Framework;
using Robust.Shared.Maths;
using Robust.Shared.Spatial;

namespace Robust.UnitTesting.Shared.Spatial;

[TestFixture]
public sealed class SpatialTileChunkTest
{
    [Test]
    public void NegativeCoordinatesAndIntegerExtremesRoundTripWithoutWrapping()
    {
        var layer = new LayerId(83);
        var values = new[] { int.MinValue, -1000001, -17, -16, -15, -1, 0, 1, 15, 16, 17, int.MaxValue };
        foreach (var x in values)
        foreach (var y in values)
        {
            var cell = new Vector2i(x, y);
            var key = SpatialChunkKey.FromCell(layer, cell);
            var local = SpatialChunkKey.LocalCell(cell);
            Assert.That(key.Indices.X, Is.EqualTo((int) Math.Floor(x / 16d)));
            Assert.That(key.Indices.Y, Is.EqualTo((int) Math.Floor(y / 16d)));
            Assert.That(key.FrameCell(local), Is.EqualTo(cell));
            Assert.That(local.X, Is.InRange(0, 15));
            Assert.That(local.Y, Is.InRange(0, 15));
        }
        Assert.Throws<OverflowException>(() => new SpatialChunkKey(layer, new Vector2i(int.MaxValue, 0)).FrameCell(Vector2i.Zero));
        Assert.Throws<ArgumentOutOfRangeException>(() => SpatialChunkKey.FromCell(layer, Vector2i.Zero).FrameCell(new Vector2i(16, 0)));
    }

    [Test]
    public void SparseChunkCopiesInputAndRejectsEmptyOrInvalidStorage()
    {
        var cell = new Vector2i(15, 15);
        var tile = new SpatialFloorTile("covering", new SurfaceId(4));
        var source = new Dictionary<Vector2i, SpatialFloorTile> { [cell] = tile };
        var key = SpatialChunkKey.FromCell(new LayerId(83), new Vector2i(-1, -1));
        var chunk = new SpatialTileChunk(key, source, 9);
        source.Clear();
        Assert.That(chunk.Tiles, Has.Count.EqualTo(1));
        Assert.That(chunk.TryGetTile(cell, out var found), Is.True);
        Assert.That(found, Is.EqualTo(tile));
        Assert.That(chunk.TryGetTile(Vector2i.Zero, out _), Is.False);
        Assert.Throws<NotSupportedException>(() => ((IDictionary<Vector2i, SpatialFloorTile>) chunk.Tiles).Clear());
        Assert.Throws<ArgumentException>(() => new SpatialTileChunk(key, source));
        source[new Vector2i(-1, 0)] = tile;
        Assert.Throws<ArgumentOutOfRangeException>(() => new SpatialTileChunk(key, source));
        Assert.Throws<ArgumentException>(() => new SpatialFloorTile(" ", new SurfaceId(4)));
        source.Clear();
        source[cell] = default;
        Assert.Throws<ArgumentException>(() => new SpatialTileChunk(key, source));
    }
}
