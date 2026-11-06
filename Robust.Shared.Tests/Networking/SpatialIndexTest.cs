using System.Numerics;
using NUnit.Framework;
using Robust.Shared.Spatial.Interest;

namespace Robust.Shared.Tests.Networking;

[TestFixture]
public sealed class SpatialIndexTest
{
    [Test]
    public void MovingAndRemovingLeavesMatchesBruteForceAcrossAllThreeAxes()
    {
        using var index = new SpatialIndex3<int>();
        var expected = new Dictionary<int, (Vector3 Min, Vector3 Max)>();
        var random = new System.Random(913571);
        Vector3 Point() => new(random.Next(-100, 101), random.Next(-100, 101), random.Next(-100, 101));
        for (var step = 0; step < 4000; step++)
        {
            var key = random.Next(300);
            if (step % 4 == 0)
                Assert.That(index.Remove(key), Is.EqualTo(expected.Remove(key)));
            else
            {
                var min = Point();
                var max = min + new Vector3(random.Next(0, 20));
                index.SetBounds(key, min, max);
                expected[key] = (min, max);
            }
            if (step % 7 == 0) index.Refine(step);
            var a = Point();
            var b = a + new Vector3(40);
            var hits = new List<int>();
            index.Query(a, b, hits);
            var wanted = expected.Where(pair => pair.Value.Min.X <= b.X && pair.Value.Max.X >= a.X
                && pair.Value.Min.Y <= b.Y && pair.Value.Max.Y >= a.Y
                && pair.Value.Min.Z <= b.Z && pair.Value.Max.Z >= a.Z).Select(pair => pair.Key);
            Assert.That(hits, Is.EquivalentTo(wanted), $"Step {step}");
            Assert.That(index.Count, Is.EqualTo(expected.Count));
        }
        foreach (var (key, bounds) in expected)
        {
            Assert.That(index.TryGetBounds(key, out var min, out var max), Is.True);
            Assert.That((min, max), Is.EqualTo(bounds));
            Assert.That(index.Remove(key), Is.True);
        }
        Assert.That(index.Count, Is.Zero);
    }

    [Test]
    public void PointQueriesHeightAndInvalidUpdatesDoNotLoseEntries()
    {
        using var index = new SpatialIndex3<int>();
        index.SetBounds(1, new Vector3(-2, -2, -2), Vector3.Zero);
        index.SetBounds(2, new Vector3(0, 0, 10), new Vector3(0, 0, 10));
        Assert.Throws<ArgumentException>(() => index.SetBounds(1, new Vector3(float.NaN), Vector3.One));
        Assert.Throws<ArgumentException>(() => index.SetBounds(3, Vector3.One, Vector3.Zero));
        var hits = new List<int>();
        index.Query(Vector3.Zero, Vector3.Zero, hits);
        Assert.That(hits, Is.EqualTo(new[] { 1 }));
        hits.Clear();
        index.Query(new Vector3(-1, -1, 9), new Vector3(1, 1, 11), hits);
        Assert.That(hits, Is.EqualTo(new[] { 2 }));
        Assert.That(index.TryGetBounds(3, out _, out _), Is.False);
        index.Dispose();
        Assert.Throws<ObjectDisposedException>(() => index.Query(Vector3.Zero, Vector3.One, hits));
    }

    [Test]
    public void ParallelReadersHaveIndependentTraversalStorage()
    {
        using var index = new SpatialIndex3<int>();
        for (var i = 0; i < 4096; i++)
            index.SetBounds(i, new Vector3(i, 0, i % 32), new Vector3(i + 1, 1, i % 32 + 1));
        index.Refine(1);
        Parallel.For(0, 256, i =>
        {
            var hits = new List<int>();
            index.Query(new Vector3(i * 8, -1, -1), new Vector3(i * 8 + 4, 2, 33), hits);
            Assert.That(hits, Is.EquivalentTo(Enumerable.Range(Math.Max(0, i * 8 - 1), i == 0 ? 5 : 6)));
        });
    }
}
