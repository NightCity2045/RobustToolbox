using System.Numerics;
using NUnit.Framework;
using Robust.Client.Graphics.Spatial;
using Robust.Shared.GameObjects;
using Robust.Shared.Maths;
using Robust.Shared.Spatial;

namespace Robust.Client.Tests.Graphics;

[TestFixture]
public sealed class SpatialChunkMeshTest
{
    [Test]
    public void SplittingAcrossNegativeChunksPreservesOpeningsWindingAreaAndSourceIdentity()
    {
        var snapshot = Scene();
        var chunks = SpatialChunkMeshBuilder.Build(snapshot, 16, new SpatialMeshBuildLimits(128, 10000));
        Assert.That(chunks.Any(c => c.Key.X == -2), Is.True);
        Assert.That(chunks.Any(c => c.Key.X == 1), Is.True);
        Assert.That(chunks.Select(c => c.Key.Layer).Distinct().Count(), Is.EqualTo(2));
        double area = 0;
        foreach (var chunk in chunks)
        {
            Assert.That(chunk.Source, Is.SameAs(snapshot));
            Assert.That(chunk.Bounds.Min.X, Is.GreaterThanOrEqualTo(chunk.Key.X * 16));
            Assert.That(chunk.Bounds.Max.X, Is.LessThanOrEqualTo((chunk.Key.X + 1) * 16));
            Assert.That(chunk.Bounds.Min.Y, Is.GreaterThanOrEqualTo(chunk.Key.Y * 16));
            Assert.That(chunk.Bounds.Max.Y, Is.LessThanOrEqualTo((chunk.Key.Y + 1) * 16));
            foreach (var face in chunk.Faces)
            {
                Assert.That(face.Surface.OwnerFrame, Is.EqualTo(snapshot.OwnerFrame));
                Assert.That(face.Entity, Is.EqualTo(new EntityUid(42)));
                Assert.That(face.Revision, Is.EqualTo(2));
                for (var i = face.FirstIndex; i < face.FirstIndex + 6; i += 3)
                {
                    var a = chunk.Vertices[chunk.Indices[i]];
                    var b = chunk.Vertices[chunk.Indices[i + 1]];
                    var c = chunk.Vertices[chunk.Indices[i + 2]];
                    var cross = Vector3.Cross(b.Position - a.Position, c.Position - a.Position);
                    Assert.That(Vector3.Dot(Vector3.Normalize(cross), a.Normal), Is.GreaterThan(0.999));
                    area += cross.Length() * 0.5;
                    if (a.Normal.Z > 0 && chunk.Key.Layer == new LayerId(1))
                    {
                        var center = (a.Position + b.Position + c.Position) / 3;
                        Assert.That(center.X <= -1 || center.X >= 1 || center.Y <= -0.5f || center.Y >= 0.5f, Is.True,
                            "A floor opening may not be filled by the visual mesh.");
                        Assert.That(a.Uv, Is.EqualTo(new Vector2(a.Position.X, a.Position.Y)), "UVs retain scale across chunk seams.");
                    }
                }
            }
        }
        Assert.That(area, Is.EqualTo(SpatialSurfaceBuilder.Build(snapshot).Sum(p => p.Area)).Within(1e-4));
    }

    [Test]
    public void BudgetsAndCancellationRejectWholeBuildWithoutReturningPartialMeshes()
    {
        var snapshot = Scene();
        Assert.Throws<InvalidOperationException>(() => SpatialChunkMeshBuilder.Build(snapshot, 16, new SpatialMeshBuildLimits(1, 10000)));
        Assert.Throws<InvalidOperationException>(() => SpatialChunkMeshBuilder.Build(snapshot, 16, new SpatialMeshBuildLimits(100, 1)));
        Assert.Throws<OperationCanceledException>(() => SpatialChunkMeshBuilder.Build(snapshot, 16,
            new SpatialMeshBuildLimits(100, 10000), new CancellationToken(true)));
        Assert.Throws<ArgumentOutOfRangeException>(() => SpatialChunkMeshBuilder.Build(snapshot, 0, new SpatialMeshBuildLimits(100, 10000)));
    }

    private static SpatialGeometrySnapshot Scene()
    {
        var frame = new EntityUid(1);
        var layer = new LayerId(1);
        var upper = new LayerId(2);
        return new SpatialGeometrySnapshot(frame, 2,
            new Dictionary<LayerId, LayerDefinition> { [layer] = new(0, "Floor"), [upper] = new(5, "Roof") },
            Array.Empty<BoundarySegment>(), new[]
            {
                new SlabSurface(new SpatialSurfaceReference(frame, new SurfaceId(1)), layer,
                    new Box2(-17, -1, 17, 1), 0, 0.2f, new[] { new Box2(-1, -0.5f, 1, 0.5f) }, "metal", new EntityUid(42), 2),
                new SlabSurface(new SpatialSurfaceReference(frame, new SurfaceId(2)), upper,
                    new Box2(-2, -1, 2, 1), 0, 0.2f, materialProfile: "roof", entity: new EntityUid(42), stateRevision: 2)
            }, Array.Empty<SpatialTileChunk>());
    }
}
