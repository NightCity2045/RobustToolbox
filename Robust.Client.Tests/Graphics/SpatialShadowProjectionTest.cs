using System.Numerics;
using NUnit.Framework;
using Robust.Client.Graphics.Spatial;

namespace Robust.Client.Tests.Graphics;

[TestFixture]
public sealed class SpatialShadowProjectionTest
{
    [Test]
    public void EveryDirectionProjectsInsideItsSelectedCubeFace()
    {
        var origin = new Vector3(8, -12, 4);
        for (var x = -2; x <= 2; x++)
        for (var y = -2; y <= 2; y++)
        for (var z = -2; z <= 2; z++)
        {
            var direction = new Vector3(x, y, z);
            if (direction == Vector3.Zero) continue;
            var camera = SpatialShadowProjection.Camera(origin, SpatialShadowProjection.Face(direction), 0.05f, 20);
            Assert.That(camera.TryProject(origin + direction, out var screen), Is.True);
            Assert.That(screen.X, Is.InRange(-1e-5f, 1.00001f));
            Assert.That(screen.Y, Is.InRange(-1e-5f, 1.00001f));
            Assert.That(screen.Z, Is.InRange(-1f, 1f));
            var ray = camera.GetRay(Vector2.Clamp(new Vector2(screen.X, screen.Y), Vector2.Zero, Vector2.One));
            Assert.That(Vector3.Dot(ray.Direction, Vector3.Normalize(direction)), Is.GreaterThan(0.99999f));
        }
    }

    [Test]
    public void ShadowBudgetIncludesAllSixFacesAndRejectsInvalidProjection()
    {
        Assert.DoesNotThrow(() => new SpatialShadowSettings(2, 128, 0.05f, 0.001f).Validate(1024 * 1024));
        Assert.Throws<ArgumentOutOfRangeException>(() => new SpatialShadowSettings(4, 512, 0.05f, 0.001f).Validate(1024 * 1024));
        Assert.Throws<ArgumentOutOfRangeException>(() => new SpatialShadowSettings(5, 16, 0.05f, 0.001f).Validate(1024 * 1024));
        Assert.Throws<ArgumentOutOfRangeException>(() => new SpatialShadowSettings(1, 128, 0, 0.001f).Validate(1024 * 1024));
        Assert.Throws<ArgumentOutOfRangeException>(() => SpatialShadowProjection.Camera(Vector3.Zero, 6, 0.1f, 10));
        Assert.Throws<ArgumentException>(() => SpatialShadowProjection.Camera(Vector3.Zero, 0, 1, 1));
    }
}
