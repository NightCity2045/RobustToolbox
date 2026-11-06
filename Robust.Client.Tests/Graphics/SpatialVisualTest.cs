using System.Numerics;
using NUnit.Framework;
using Robust.Client.Graphics;
using Robust.Client.Graphics.Spatial;
using Robust.Shared.GameObjects;
using Robust.Shared.Graphics.RSI;
using Robust.Shared.Maths;
using Robust.Shared.Spatial;
using Robust.Shared.Utility;

namespace Robust.Client.Tests.Graphics;

[TestFixture]
public sealed class SpatialVisualTest
{
    private static PerspectiveCamera Camera(Vector3 position) => new(new RigidTransform3(position, Quaternion.Identity), MathF.PI / 3, 1, 0.1f, 100);

    [Test]
    public void ObserversSelectDifferentExistingDirectionsWithoutChangingCanonicalPose()
    {
        var pose = new RigidTransform3(new Vector3(5, 2, 10), Quaternion.CreateFromAxisAngle(Vector3.UnitZ, 0.7f));
        var profile = new SpatialSpriteProfile(SpatialSpriteKind.DirectionalBillboard, new Vector2(0.8f, 1.8f), new Vector2(0.5f, 0), MathF.PI / 3);
        foreach (var (offset, direction) in new[] { (-Vector3.UnitY, RsiDirection.South), (Vector3.UnitY, RsiDirection.North),
                     (Vector3.UnitX, RsiDirection.East), (-Vector3.UnitX, RsiDirection.West),
                     (new Vector3(1, -1, 0), RsiDirection.SouthEast) })
        {
            var view = Camera(pose.TransformPoint(offset * 10));
            Assert.That(SpatialSpriteAnimation.Direction(pose, view, RsiDirectionType.Dir8), Is.EqualTo(direction));
            var visual = profile.Resolve(pose, view, "sprite");
            var expected = Vector3.Normalize(view.Pose.Position - pose.Position);
            Assert.That(Vector3.Dot(visual.Pose.TransformVector(Vector3.UnitZ), expected), Is.GreaterThan(0.99999f));
            Assert.That(Vector3.Distance(visual.Pose.TransformPoint(new Vector3(0, -0.9f, 0)), pose.Position), Is.LessThan(1e-5f));
            Assert.That(visual.Size, Is.EqualTo(new Vector3(0.8f, 1.8f, 1)));
            Assert.That(SpatialSpriteAnimation.Direction(pose, view, RsiDirectionType.Dir1), Is.EqualTo(RsiDirection.South));
        }
        Assert.That(pose.Position, Is.EqualTo(new Vector3(5, 2, 10)));
    }

    [Test]
    public void SteepViewsKeepFiniteBoundedTiltAndFixedPlaneProfilesIgnoreObserver()
    {
        var pose = new RigidTransform3(new Vector3(1, 2, 3), Quaternion.CreateFromYawPitchRoll(0.4f, 0.3f, 0.2f));
        var profile = new SpatialSpriteProfile(SpatialSpriteKind.DirectionalBillboard, new Vector2(1, 2), new Vector2(0.5f), 0.5f);
        foreach (var height in new[] { -100f, 100f })
        {
            var view = Camera(pose.TransformPoint(new Vector3(0, 0, height)));
            var visual = profile.Resolve(pose, view, "sprite");
            Assert.That(visual.Pose.IsValid, Is.True);
            var tilt = Vector3.Dot(visual.Pose.TransformVector(Vector3.UnitZ), pose.TransformVector(Vector3.UnitZ));
            Assert.That(tilt, Is.EqualTo(MathF.Sin(0.5f) * MathF.Sign(height)).Within(1e-5));
            foreach (var kind in new[] { SpatialSpriteKind.GroundSprite, SpatialSpriteKind.SurfaceQuad })
                Assert.That((profile with { Kind = kind }).Resolve(pose, view, "sprite").Pose, Is.EqualTo(pose));
        }
        Assert.Throws<ArgumentException>(() => (profile with { Size = new Vector2(0, 1) }).Resolve(pose, Camera(Vector3.Zero), "sprite"));
    }

    [Test]
    public void AnimationUsesExistingFoldedTimingAndExplicitLoopPolicy()
    {
        var rsi = new RSI(new Vector2i(32, 32), new ResPath("/Textures/debug.rsi"));
        var state = new RSI.State(rsi.Size, rsi, new RSI.StateId("sample"), RsiDirectionType.Dir4,
            new[] { 0.25f, 0.5f, 0.25f }, Array.Empty<AtlasTexture[]>());
        Assert.That(SpatialSpriteAnimation.Frame(state, 0), Is.Zero);
        Assert.That(SpatialSpriteAnimation.Frame(state, 0.25), Is.EqualTo(1));
        Assert.That(SpatialSpriteAnimation.Frame(state, 0.75), Is.EqualTo(2));
        Assert.That(SpatialSpriteAnimation.Frame(state, 1000.25), Is.EqualTo(1));
        Assert.That(SpatialSpriteAnimation.Frame(state, 1000.25, false), Is.EqualTo(2));
        Assert.Throws<ArgumentOutOfRangeException>(() => SpatialSpriteAnimation.Frame(state, double.NaN));
        Assert.That(state.Delays, Is.EqualTo(new[] { 0.25f, 0.5f, 0.25f }));
    }

    [Test]
    public void DecalTracksMovingSurfaceAndRejectsHolesAndStaleRevisions()
    {
        var frame = new EntityUid(10); var layer = new LayerId(1); var reference = new SpatialSurfaceReference(frame, new SurfaceId(1));
        var snapshot = new SpatialGeometrySnapshot(frame, 3, new Dictionary<LayerId, LayerDefinition> { [layer] = new(0, "floor") },
            Array.Empty<BoundarySegment>(), new[] { new SlabSurface(reference, layer, new Box2(-2, -2, 2, 2), 0, 0.2f,
                new[] { new Box2(-0.5f, -0.5f, 0.5f, 0.5f) }, "floor", stateRevision: 3) }, Array.Empty<SpatialTileChunk>());
        var attachment = new SpatialDecalAttachment(reference, 3, SpatialFaceDirection.PositiveZ, 0, new Box2(1, 1, 1.5f, 1.5f), 0.001f);
        var pose = new RigidTransform3(new Vector3(5, 8, 12), Quaternion.CreateFromYawPitchRoll(0.3f, 0.2f, 1));
        Assert.That(attachment.TryResolve(snapshot, pose, "decal", out var visual), Is.True);
        Assert.That(Vector3.Distance(visual!.Pose.Position, pose.TransformPoint(new Vector3(1.25f, 1.25f, 0.001f))), Is.LessThan(1e-5));
        Assert.That(visual.Surface, Is.EqualTo(reference));
        Assert.That((attachment with { Revision = 2 }).TryResolve(snapshot, pose, "decal", out _), Is.False);
        Assert.That((attachment with { Region = new Box2(-0.2f, -0.2f, 0.2f, 0.2f) }).TryResolve(snapshot, pose, "decal", out _), Is.False);
        Assert.That((attachment with { Plane = 10 }).TryResolve(snapshot, pose, "decal", out _), Is.False);
    }
}
