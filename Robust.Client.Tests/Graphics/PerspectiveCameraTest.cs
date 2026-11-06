using System.Numerics;
using NUnit.Framework;
using Robust.Client.Graphics.Spatial;
using Robust.Shared.Maths;

namespace Robust.Client.Tests.Graphics;

[TestFixture]
public sealed class PerspectiveCameraTest
{
    [Test]
    public void ProjectionAndPickingAgreeAtCenterCornersAndClipPlanes()
    {
        var pose = new RigidTransform3(new Vector3(15, -7, 9),
            Quaternion.CreateFromYawPitchRoll(0.7f, -0.3f, 0.4f));
        var camera = new PerspectiveCamera(pose, MathF.PI / 2, 16f / 9, 0.1f, 100);
        foreach (var point in new[] { Vector2.Zero, Vector2.One, new Vector2(0, 1), new Vector2(1, 0), new Vector2(0.5f) })
        {
            var ray = camera.GetRay(point);
            Assert.That(ray.Direction.Length(), Is.EqualTo(1).Within(1e-5));
            foreach (var distance in new[] { ray.MinimumDistance, (ray.MinimumDistance + ray.MaximumDistance) * 0.5f, ray.MaximumDistance })
            {
                Assert.That(camera.TryProject(ray.Origin + ray.Direction * distance, out var projected), Is.True);
                Assert.That(Vector2.Distance(new Vector2(projected.X, projected.Y), point), Is.LessThan(2e-5));
                if (distance == ray.MinimumDistance) Assert.That(projected.Z, Is.EqualTo(-1).Within(5e-5));
                if (distance == ray.MaximumDistance) Assert.That(projected.Z, Is.EqualTo(1).Within(5e-5));
            }
        }
        Assert.That(camera.TryProject(pose.Position - camera.Forward, out _), Is.False);
        Assert.That(Vector3.Distance(camera.GetRay(new Vector2(0.5f)).Direction, camera.Forward), Is.LessThan(1e-5));
    }

    [Test]
    public void FrustumRejectsBehindNearFarAndSidesButIncludesCrossingGeometry()
    {
        var camera = new PerspectiveCamera(RigidTransform3.Identity, MathF.PI / 2, 1, 1, 20);
        Assert.That(camera.Frustum.Intersects(new Box3(new Vector3(-1, 2, -1), new Vector3(1, 3, 1))), Is.True);
        Assert.That(camera.Frustum.Intersects(new Box3(new Vector3(-1, -3, -1), new Vector3(1, -2, 1))), Is.False);
        Assert.That(camera.Frustum.Intersects(new Box3(new Vector3(-0.1f, 0.1f, -0.1f), new Vector3(0.1f, 0.5f, 0.1f))), Is.False);
        Assert.That(camera.Frustum.Intersects(new Box3(new Vector3(-1, 21, -1), new Vector3(1, 22, 1))), Is.False);
        Assert.That(camera.Frustum.Intersects(new Box3(new Vector3(5, 2, -1), new Vector3(6, 3, 1))), Is.False);
        Assert.That(camera.Frustum.Intersects(new Box3(new Vector3(-100, -100, -100), new Vector3(100))), Is.True);
        var chunk = new Box3(new Vector3(-0.5f), new Vector3(0.5f));
        Assert.That(camera.Frustum.Intersects(chunk, new RigidTransform3(Vector3.UnitY * 5,
            Quaternion.CreateFromAxisAngle(Vector3.UnitZ, 0.4f))), Is.True);
        Assert.That(camera.Frustum.Intersects(chunk, new RigidTransform3(-Vector3.UnitY * 5, Quaternion.Identity)), Is.False);
    }

    [Test]
    public void LargeCameraTranslationRetainsDirectionAndInvalidProfilesFailExplicitly()
    {
        var camera = new PerspectiveCamera(new RigidTransform3(new Vector3(100000000, 0, 0), Quaternion.Identity), 1, 1, 1, 100);
        Assert.That(camera.Forward, Is.EqualTo(Vector3.UnitY));
        Assert.That(camera.GetRay(new Vector2(0.5f)).Direction, Is.EqualTo(Vector3.UnitY));
        Assert.Throws<ArgumentException>(() => new PerspectiveCamera(default, 1, 1, 1, 100));
        Assert.Throws<ArgumentException>(() => new PerspectiveCamera(RigidTransform3.Identity, 0, 1, 1, 100));
        Assert.Throws<ArgumentException>(() => new PerspectiveCamera(RigidTransform3.Identity, 1, 0, 1, 100));
        Assert.Throws<ArgumentException>(() => new PerspectiveCamera(RigidTransform3.Identity, 1, 1, 1, 1));
        Assert.Throws<ArgumentException>(() => new PerspectiveCamera(RigidTransform3.Identity, 1, 1, 1, float.PositiveInfinity));
        Assert.Throws<ArgumentException>(() => camera.GetRay(new Vector2(float.NaN, 0)));
    }
}
