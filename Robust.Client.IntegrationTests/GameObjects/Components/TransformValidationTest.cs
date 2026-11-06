using System.Numerics;
using NUnit.Framework;
using Robust.Shared.GameObjects;
using Robust.Shared.Map;
using Robust.Shared.Maths;
using Robust.Shared.Network;
using Robust.Shared.Timing;
using Robust.UnitTesting;

namespace Robust.Client.IntegrationTests.GameObjects.Components;

[TestFixture]
public sealed class TransformValidationTest : RobustIntegrationTest
{
    [Test]
    public async Task InvalidPredictionDoesNotStartInterpolation()
    {
        using var server = StartServer(new ServerIntegrationOptions { Pool = false });
        using var client = StartClient(new ClientIntegrationOptions { Pool = false });
        await Task.WhenAll(server.WaitIdleAsync(), client.WaitIdleAsync());
        client.SetConnectTarget(server);
        await client.WaitPost(() => client.Resolve<IClientNetManager>().ClientConnect(null!, 0, null!));
        await RunTicksSync(server, client, 10);
        await client.WaitAssertion(() =>
        {
            var entities = client.Resolve<IEntityManager>();
            entities.System<SharedMapSystem>().CreateMap(out var mapId);
            var uid = entities.Spawn(null, new MapCoordinates(Vector2.One, mapId));
            var transform = entities.GetComponent<TransformComponent>(uid);
            var transforms = entities.System<SharedTransformSystem>();
            var coords = transform.Coordinates;
            var nextPosition = transform.NextPosition;
            var nextRotation = transform.NextRotation;
            var prevPosition = transform.PrevPosition;
            var prevRotation = transform.PrevRotation;
            var lerping = transform.ActivelyLerping;
            var lastLerp = transform.LastLerp;

            Assert.Throws<ArgumentOutOfRangeException>(() => transforms.SetLocalPosition(uid, new Vector2(float.NaN, 0)));
            Assert.Throws<ArgumentOutOfRangeException>(() => transforms.SetLocalRotation(uid, new Angle(double.PositiveInfinity)));
            Assert.Throws<ArgumentOutOfRangeException>(() => transforms.SetLocalPositionRotation(uid, Vector2.Zero, new Angle(double.NaN)));
            Assert.Multiple(() =>
            {
                Assert.That(transform.Coordinates, Is.EqualTo(coords));
                Assert.That(transform.LocalRotation, Is.EqualTo(Angle.Zero));
                Assert.That(transform.NextPosition, Is.EqualTo(nextPosition));
                Assert.That(transform.NextRotation, Is.EqualTo(nextRotation));
                Assert.That(transform.PrevPosition, Is.EqualTo(prevPosition));
                Assert.That(transform.PrevRotation, Is.EqualTo(prevRotation));
                Assert.That(transform.ActivelyLerping, Is.EqualTo(lerping));
                Assert.That(transform.LastLerp, Is.EqualTo(lastLerp));
            });
        });
    }

    [Test]
    public async Task SpatialInterpolationIncludesHeightAndPitch()
    {
        using var server = StartServer(new ServerIntegrationOptions { Pool = false });
        using var client = StartClient(new ClientIntegrationOptions { Pool = false });
        await Task.WhenAll(server.WaitIdleAsync(), client.WaitIdleAsync());
        client.SetConnectTarget(server);
        await client.WaitPost(() => client.Resolve<IClientNetManager>().ClientConnect(null!, 0, null!));
        await RunTicksSync(server, client, 10);
        await client.WaitAssertion(() =>
        {
            var entities = client.Resolve<IEntityManager>();
            entities.System<SharedMapSystem>().CreateMap(out var mapId);
            var uid = entities.Spawn(null, new MapCoordinates(Vector2.Zero, mapId));
            var transform = entities.GetComponent<TransformComponent>(uid);
            var transforms = entities.System<SharedTransformSystem>();
            var timing = client.Resolve<IGameTiming>();
            transforms.SetLocalPoseNoLerp(uid, new RigidTransform3(new Vector3(0, 0, 2), Quaternion.Identity));
            transforms.SetLocalPose(uid, new RigidTransform3(new Vector3(0, 0, 3),
                Quaternion.CreateFromAxisAngle(Vector3.UnitX, MathF.PI / 2)));
            timing.TickRemainder = timing.TickPeriod / 2;
            transforms.FrameUpdate(0);
            Assert.That(transform.LocalPose.Position.Z, Is.EqualTo(2.5f).Within(1e-5f));
            var up = transform.LocalPose.TransformVector(Vector3.UnitY);
            Assert.That(Vector3.Distance(up, new Vector3(0, MathF.Sqrt(0.5f), MathF.Sqrt(0.5f))), Is.LessThan(1e-5f));
            Assert.That(transform.NextPosition!.Value.Z, Is.EqualTo(3));
            timing.TickRemainder = TimeSpan.Zero;
        });
    }

}
