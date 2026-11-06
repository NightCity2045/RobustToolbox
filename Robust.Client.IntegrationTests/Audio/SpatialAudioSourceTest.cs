using System.Numerics;
using NUnit.Framework;
using OpenTK.Audio.OpenAL;
using Robust.Client.Audio;
using Robust.Shared.Audio.Components;
using Robust.Shared.IoC;
using Robust.Shared.Maths;
using Robust.UnitTesting;

namespace Robust.Client.IntegrationTests.Audio;

/// <summary>Use ALSOFT_DRIVERS=null for actual OpenAL qualification without an output device.</summary>
[TestFixture, Explicit("Requires OpenAL; CI runs this fixture explicitly with the null output driver.")]
public sealed class SpatialAudioSourceTest : RobustIntegrationTest
{
    [Test]
    public async Task NativeSourcesAndListenerKeepHeightVelocityAndFullOrientation()
    {
        var client = StartClient(new ClientIntegrationOptions
        {
            Pool = false,
            InitIoC = () =>
            {
                IoCManager.Register<IAudioManager, AudioManager>(overwrite: true);
                IoCManager.Register<IAudioInternal, AudioManager>(overwrite: true);
            }
        });
        await client.WaitIdleAsync();
        await client.WaitAssertion(() =>
        {
            var audio = client.ResolveDependency<IAudioInternal>();
            using var stream = audio.LoadAudioRaw(new short[2205], 1, 22050, "xyz-test");
            using var source = audio.CreateAudioSource(stream)!;
            using var other = audio.CreateAudioSource(stream)!;
            Assert.That(source, Is.Not.Null); Assert.That(other, Is.Not.Null);
            var position = new Vector3(1, 2, 30); var velocity = new Vector3(4, 5, -6);
            source.Position3 = position; source.Velocity3 = velocity;
            Assert.That(source.Position3, Is.EqualTo(position)); Assert.That(source.Velocity3, Is.EqualTo(velocity));
            Assert.That(other.Position3, Is.EqualTo(Vector3.Zero));
            source.Position = new Vector2(7, 8); source.Velocity = new Vector2(9, 10);
            Assert.That(source.Position3, Is.EqualTo(new Vector3(7, 8, 30)));
            Assert.That(source.Velocity3, Is.EqualTo(new Vector3(9, 10, -6)));
            Assert.Throws<ArgumentOutOfRangeException>(() => source.Position3 = new Vector3(0, 0, float.NaN));
            Assert.Throws<ArgumentOutOfRangeException>(() => source.Velocity3 = new Vector3(float.PositiveInfinity, 0, 0));
            Assert.That(source.Position3.Z, Is.EqualTo(30)); Assert.That(source.Velocity3.Z, Is.EqualTo(-6));

            // AudioComponent forwards derived state, not a second networked pose.
            var component = new AudioComponent();
            component.Position3 = position; component.Velocity3 = velocity;
            Assert.That(component.Source.Position3, Is.EqualTo(position)); Assert.That(component.Velocity3, Is.EqualTo(velocity));
            using var buffered = audio.CreateBufferedAudioSource(2)!;
            buffered.Position3 = new Vector3(1, 1, 12); buffered.Velocity3 = new Vector3(0, 0, 3);
            Assert.That(buffered.Position3.Z, Is.EqualTo(12)); Assert.That(buffered.Velocity3.Z, Is.EqualTo(3));

            var listener = new SpatialAudioListener(new RigidTransform3(new Vector3(-4, 7, 19),
                Quaternion.CreateFromYawPitchRoll(0.5f, -0.6f, 0.3f)), new Vector3(1, -2, 3));
            client.EntMan.Startup();
            audio.SetZOffset(100);
            var system = client.EntMan.System<AudioSystem>();
            system.SpatialListenerProvider = () => listener;
            system.FrameUpdate(1);
            AL.GetListener(ALListener3f.Position, out var x, out var y, out var z);
            Assert.That(new Vector3(x, y, z), Is.EqualTo(listener.Pose.Position));
            AL.GetListener(ALListener3f.Velocity, out x, out y, out z);
            Assert.That(new Vector3(x, y, z), Is.EqualTo(listener.Velocity));
            AL.GetListener(ALListenerfv.Orientation, out OpenTK.Mathematics.Vector3 forward, out OpenTK.Mathematics.Vector3 up);
            Assert.That(Vector3.Distance(new Vector3(forward.X, forward.Y, forward.Z), listener.Forward), Is.LessThan(1e-5));
            Assert.That(Vector3.Distance(new Vector3(up.X, up.Y, up.Z), listener.Up), Is.LessThan(1e-5));
            Assert.Throws<ArgumentException>(() => audio.SetSpatialListener(listener with { Velocity = new Vector3(float.NaN) }));
            AL.GetListener(ALListener3f.Position, out x, out y, out z);
            Assert.That(z, Is.EqualTo(19), "Invalid listener must be rejected before native state changes.");
            system.SpatialListenerProvider = null;
            system.FrameUpdate(1);
            AL.GetListener(ALListener3f.Position, out x, out y, out z);
            Assert.That(z, Is.EqualTo(100), "Returning to flat Eye restores its existing Z offset.");
            Assert.That(system.CurrentSpatialListener, Is.Null);
            source.Dispose();
            source.Dispose();
            Assert.Throws<ObjectDisposedException>(() => _ = source.Position3);
            Assert.That(other.Position3, Is.EqualTo(Vector3.Zero));
            var deferred = audio.CreateAudioSource(stream)!;
            Task.Run(deferred.Dispose).GetAwaiter().GetResult();
            Assert.Throws<ObjectDisposedException>(() => _ = deferred.Position3);
            audio.FlushALDisposeQueues();
            deferred.Dispose();
            Assert.That(AL.GetError(), Is.EqualTo(ALError.NoError));
        });
    }
}
