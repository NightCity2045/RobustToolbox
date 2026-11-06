using System.Numerics;
using NUnit.Framework;
using Robust.Client.Graphics.Spatial;
using Robust.Shared.GameObjects;
using Robust.Shared.Maths;
using Robust.Shared.Spatial;

namespace Robust.Client.Tests.Graphics;

[TestFixture]
public sealed class TopDownSliceViewTest
{
    [Test]
    public void ProjectionUsesSelectedMovingFrameAndPickingPreservesFloorOpening()
    {
        var frame = new EntityUid(1);
        var lower = new LayerId(1); var upper = new LayerId(2);
        var snapshot = new SpatialGeometrySnapshot(frame, 0,
            new Dictionary<LayerId, LayerDefinition> { [lower] = new(0, "Lower"), [upper] = new(3, "Upper") },
            Array.Empty<BoundarySegment>(), new[]
            {
                new SlabSurface(new SpatialSurfaceReference(frame, new SurfaceId(1)), lower, new Box2(-4, -4, 4, 4), 0, 0.2f,
                    materialProfile: "floor"),
                new SlabSurface(new SpatialSurfaceReference(frame, new SurfaceId(2)), upper, new Box2(-4, -4, 4, 4), 0, 0.2f,
                    new[] { new Box2(-1, -1, 1, 1) }, "floor")
            }, Array.Empty<SpatialTileChunk>());
        var pose = new RigidTransform3(new Vector3(30, -20, 8), Quaternion.CreateFromYawPitchRoll(0.3f, -0.5f, 1));
        var view = new TopDownSliceView(new SpatialLayerReference(frame, upper), pose, Vector2.Zero, new Vector2i(800, 600), 50, 4);
        var pixel = view.ProjectLocal(new Vector2(2, 1));
        Assert.That(pixel, Is.EqualTo(new Vector2(500, 250)));
        Assert.That(Vector2.Distance(view.ProjectWorld(view.UnprojectWorld(pixel)), pixel), Is.LessThan(0.001));
        Assert.That(Vector3.Distance(view.UnprojectWorld(pixel), pose.TransformPoint(new Vector3(2, 1, 4))), Is.LessThan(0.001));
        Assert.That(view.Includes(new SpatialLayerReference(frame, lower)), Is.False);
        Assert.That(view.Includes(new SpatialLayerReference(new EntityUid(2), upper)), Is.False);
        Assert.That(view.GetSurfaces(snapshot).All(s => s.Surface.Surface == new SurfaceId(2)), Is.True);
        Assert.That(view.PickSurface(snapshot, new Vector2(400, 300)), Is.Null, "Upper opening must not select the lower floor.");
        Assert.That(view.PickSurface(snapshot, pixel)!.Value.Surface.Surface, Is.EqualTo(new SurfaceId(2)));
        Assert.That(view.PickSurface(snapshot, new Vector2(-1, 300)), Is.Null);
    }

    [Test]
    public void HeightCutAndSelectedLayerAreExplicit()
    {
        var frame = new EntityUid(1); var layer = new LayerId(1);
        var snapshot = new SpatialGeometrySnapshot(frame, 0, new Dictionary<LayerId, LayerDefinition> { [layer] = new(3, "Upper") },
            Array.Empty<BoundarySegment>(), new[]
            {
                new SlabSurface(new SpatialSurfaceReference(frame, new SurfaceId(1)), layer, new Box2(-1, -1, 1, 1), 0, 0.2f)
            }, Array.Empty<SpatialTileChunk>());
        var view = new TopDownSliceView(new SpatialLayerReference(frame, layer), RigidTransform3.Identity,
            Vector2.Zero, new Vector2i(320, 240), 32, 2.5f);
        Assert.That(view.GetSurfaces(snapshot), Is.Empty);
        var wrong = new TopDownSliceView(new SpatialLayerReference(frame, new LayerId(9)), RigidTransform3.Identity,
            Vector2.Zero, new Vector2i(320, 240), 32, 4);
        Assert.Throws<ArgumentException>(() => wrong.GetSurfaces(snapshot));
        Assert.Throws<ArgumentException>(() => new TopDownSliceView(new SpatialLayerReference(frame, layer), RigidTransform3.Identity,
            Vector2.Zero, new Vector2i(320, 240), 0, 4));
    }
}
