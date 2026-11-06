using System.Numerics;
using NUnit.Framework;
using Robust.Client.Graphics.Spatial;
using Robust.Shared.GameObjects;
using Robust.Shared.Maths;
using Robust.Shared.Spatial;

namespace Robust.Client.Tests.Graphics;

[TestFixture]
public sealed class SpatialMeshCacheTest
{
    private static readonly SpatialMeshBuildLimits MeshLimits = new(128, 10000);
    private static readonly SpatialMeshCacheLimits CacheLimits = new(8, 2, 8 * 1024 * 1024, MeshLimits);

    [Test]
    public void IncrementalJobsPreserveUnchangedBuffersAndRebuildOcclusionAcrossChunkEdges()
    {
        using var cache = new SpatialMeshCache(16, CacheLimits);
        var original = Scene(1, false);
        cache.Request(original);
        var first = Complete(cache, original);
        var far = first.Single(m => m.Key.X == 4);
        var added = Scene(2, true);
        cache.Request(added);
        Assert.That(cache.TryGet(original, out _), Is.False, "Old walls are hidden as soon as a new revision is requested.");
        Assert.That(cache.TryGet(added, out _), Is.False);
        var second = Complete(cache, added);
        Assert.That(second.Single(m => m.Key.X == 4).Vertices, Is.SameAs(far.Vertices));
        CheckAgainstFullBuild(added, second);
        var removed = Scene(3, false);
        cache.Request(removed);
        var third = Complete(cache, removed);
        Assert.That(third.Single(m => m.Key.X == 4).Vertices, Is.SameAs(far.Vertices));
        CheckAgainstFullBuild(removed, third);
        Assert.That(cache.ResidentBytes, Is.EqualTo(third.Sum(m => m.EstimatedCpuBytes)));
    }

    [Test]
    public void SupersededAndRemovedJobsCannotInstallIntoReplacementFrame()
    {
        using var cache = new SpatialMeshCache(16, CacheLimits);
        var old = Scene(1, false);
        cache.Request(old); cache.Pump();
        Assert.That(cache.RunningJobs, Is.EqualTo(1));
        var changed = Scene(2, true);
        cache.Request(changed);
        cache.Remove(old.OwnerFrame);
        // Same UID and revision are insufficient: only the actual current request
        // may publish after a world/frame lifetime has been replaced.
        var replacement = Scene(1, true);
        cache.Request(replacement);
        var result = Complete(cache, replacement);
        Assert.That(result.All(m => ReferenceEquals(m.Source, replacement)), Is.True);
        Assert.That(cache.TryGet(old, out _), Is.False);
        Assert.That(cache.TryGet(changed, out _), Is.False);
        CheckAgainstFullBuild(replacement, result);
        cache.Remove(replacement.OwnerFrame);
        Assert.That(cache.ResidentBytes, Is.Zero);
        cache.Request(old); cache.Pump(); cache.Dispose();
        Assert.Throws<ObjectDisposedException>(() => cache.Pump());
    }

    [Test]
    public void ResidentAndFrameBudgetsDoNotPublishPartialGeometry()
    {
        using var cache = new SpatialMeshCache(16, new SpatialMeshCacheLimits(1, 1, 1, MeshLimits));
        var source = Scene(1, false);
        cache.Request(source);
        Assert.Throws<InvalidOperationException>(() => cache.Request(Scene(1, false, new EntityUid(2))));
        Wait(cache, source);
        Assert.That(cache.GetStatus(source.OwnerFrame, out _), Is.EqualTo(SpatialMeshStatus.BudgetExceeded));
        Assert.That(cache.ResidentBytes, Is.Zero);
        Assert.That(cache.TryGet(source, out _), Is.False);
    }

    private static IReadOnlyList<SpatialChunkMesh> Complete(SpatialMeshCache cache, SpatialGeometrySnapshot source)
    {
        Wait(cache, source);
        Assert.That(cache.GetStatus(source.OwnerFrame, out var error), Is.EqualTo(SpatialMeshStatus.Ready), error?.ToString());
        Assert.That(cache.TryGet(source, out var meshes), Is.True);
        return meshes!;
    }

    private static void Wait(SpatialMeshCache cache, SpatialGeometrySnapshot source)
    {
        Assert.That(SpinWait.SpinUntil(() =>
        {
            cache.Pump();
            Assert.That(cache.RunningJobs, Is.LessThanOrEqualTo(CacheLimits.MaximumConcurrentBuilds));
            return cache.GetStatus(source.OwnerFrame, out _) != SpatialMeshStatus.Pending;
        }, TimeSpan.FromSeconds(15)), Is.True, "Bounded CPU mesh worker did not finish.");
    }

    private static void CheckAgainstFullBuild(SpatialGeometrySnapshot source, IReadOnlyList<SpatialChunkMesh> actual)
    {
        var expected = Areas(SpatialChunkMeshBuilder.Build(source, 16, MeshLimits));
        var measured = Areas(actual);
        Assert.That(measured.Keys, Is.EquivalentTo(expected.Keys));
        foreach (var (key, area) in expected) Assert.That(measured[key], Is.EqualTo(area).Within(1e-5));
        Assert.That(actual.All(m => ReferenceEquals(m.Source, source)), Is.True);

        static Dictionary<(SpatialMeshChunkKey, SpatialSurfaceReference, SpatialFaceDirection), double> Areas(IEnumerable<SpatialChunkMesh> meshes)
        {
            var result = new Dictionary<(SpatialMeshChunkKey, SpatialSurfaceReference, SpatialFaceDirection), double>();
            foreach (var mesh in meshes)
            foreach (var face in mesh.Faces)
            {
                var key = (mesh.Key, face.Surface, face.Direction);
                var area = result.GetValueOrDefault(key);
                for (var i = face.FirstIndex; i < face.FirstIndex + 6; i += 3)
                {
                    var a = mesh.Vertices[mesh.Indices[i]].Position;
                    var b = mesh.Vertices[mesh.Indices[i + 1]].Position;
                    var c = mesh.Vertices[mesh.Indices[i + 2]].Position;
                    area += Vector3.Cross(b - a, c - a).Length() * 0.5;
                }
                result[key] = area;
            }
            return result;
        }
    }

    private static SpatialGeometrySnapshot Scene(ulong revision, bool obstacle, EntityUid? owner = null)
    {
        var frame = owner ?? new EntityUid(1);
        var layer = new LayerId(1);
        var slabs = new List<SlabSurface>
        {
            new(new SpatialSurfaceReference(frame, new SurfaceId(1)), layer, new Box2(-1, -1, 31, 3), 0, 0.25f, materialProfile: "floor", stateRevision: 1),
            new(new SpatialSurfaceReference(frame, new SurfaceId(2)), layer, new Box2(64, 0, 66, 2), 0, 0.25f, materialProfile: "floor", stateRevision: 1)
        };
        if (obstacle) slabs.Add(new SlabSurface(new SpatialSurfaceReference(frame, new SurfaceId(3)), layer,
            new Box2(15, -1, 17, 1), 2, 2, materialProfile: "obstacle", stateRevision: revision));
        return new SpatialGeometrySnapshot(frame, revision, new Dictionary<LayerId, LayerDefinition> { [layer] = new(0, "Floor") },
            Array.Empty<BoundarySegment>(), slabs, Array.Empty<SpatialTileChunk>());
    }
}
