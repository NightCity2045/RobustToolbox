using System.Numerics;
using NUnit.Framework;
using Robust.Client.Audio;
using Robust.Shared.Maths;

namespace Robust.Client.Tests.Audio;

[TestFixture]
public sealed class SpatialAudioTest
{
    [Test]
    public void HeadlessSourcesAreIndependentAndKeepXYZCompatibilitySemantics()
    {
        var audio = new HeadlessAudioManager();
        using var stream = audio.LoadAudioRaw(new short[100], 1, 22050);
        using var first = audio.CreateAudioSource(stream);
        using var second = audio.CreateAudioSource(stream);
        first.Position3 = new Vector3(1, 2, 30); first.Velocity3 = new Vector3(4, 5, 6);
        first.Position = new Vector2(-1, -2); first.Velocity = new Vector2(-3, -4);
        Assert.That(first.Position3, Is.EqualTo(new Vector3(-1, -2, 30)));
        Assert.That(first.Velocity3, Is.EqualTo(new Vector3(-3, -4, 6)));
        Assert.That(second.Position3, Is.EqualTo(Vector3.Zero)); Assert.That(second.Velocity3, Is.EqualTo(Vector3.Zero));
        Assert.Throws<ArgumentOutOfRangeException>(() => first.Position3 = new Vector3(0, 0, float.NaN));
        Assert.Throws<ArgumentOutOfRangeException>(() => first.Velocity3 = new Vector3(0, float.PositiveInfinity, 0));
        Assert.That(first.Position3.Z, Is.EqualTo(30)); Assert.That(first.Velocity3.Z, Is.EqualTo(6));
        using var buffered = audio.CreateBufferedAudioSource(2)!;
        using var otherBuffer = audio.CreateBufferedAudioSource(2)!;
        buffered.Position3 = new Vector3(1, 2, 3);
        Assert.That(otherBuffer.Position3, Is.EqualTo(Vector3.Zero));
    }

    [Test]
    public void ListenerFollowsFullCanonicalOrientationAndRejectsInvalidInputs()
    {
        var pose = new RigidTransform3(new Vector3(10, -20, 40), Quaternion.CreateFromYawPitchRoll(0.5f, -0.6f, 0.3f));
        var listener = new SpatialAudioListener(pose, new Vector3(1, 2, 3));
        var audio = new HeadlessAudioManager();
        Assert.DoesNotThrow(() => audio.SetSpatialListener(listener));
        Assert.That(Vector3.Dot(listener.Forward, listener.Up), Is.EqualTo(0).Within(1e-6));
        Assert.That(Vector3.Distance(Vector3.Cross(listener.Forward, listener.Up), pose.TransformVector(Vector3.UnitX)), Is.LessThan(1e-6));
        Assert.Throws<ArgumentException>(() => audio.SetSpatialListener(default));
        Assert.Throws<ArgumentException>(() => audio.SetSpatialListener(listener with { Velocity = new Vector3(float.NaN) }));
    }
}
