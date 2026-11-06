using System.Numerics;
using NUnit.Framework;

namespace Robust.Shared.Maths.Tests;

[TestFixture]
public sealed class Box3Test
{
    [Test]
    public void OverlapAndContainmentRespectHeightAndTouchingFaces()
    {
        var lower = new Box3(new Vector3(-1, -2, -3), new Vector3(1, 2, 0));
        var upper = new Box3(new Vector3(-1, -2, 1), new Vector3(1, 2, 4));
        var touching = new Box3(new Vector3(-1, -2, 0), new Vector3(1, 2, 4));
        Assert.That(lower.Intersects(upper), Is.False);
        Assert.That(lower.Intersects(touching), Is.True);
        Assert.That(lower.Contains(new Vector3(0, 0, -1)), Is.True);
        Assert.That(lower.Contains(new Vector3(0, 0, 1)), Is.False);
        Assert.That(lower.Contains(new Vector3(float.NaN, 0, -1)), Is.False);
        Assert.That(lower.Volume, Is.EqualTo(24));
        Assert.That(lower.Center, Is.EqualTo(new Vector3(0, 0, -1.5f)));
        Assert.That(default(Box3).Intersects(lower), Is.False);
    }
}
