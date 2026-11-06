using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Threading.Tasks;
using NUnit.Framework;
using Robust.Client.GameStates;
using Robust.Client.Spatial;
using Robust.Shared;
using Robust.Shared.GameObjects;
using Robust.Shared.Maths;
using Robust.Shared.Spatial;
using Robust.Shared.Spatial.Networking;
using Robust.Shared.Spatial.Physics;

namespace Robust.UnitTesting.Server.GameStates;

[TestFixture]
public sealed class SpatialPoseHistoryTest : RobustIntegrationTest
{
    [TestCase(true)]
    [TestCase(false)]
    public async Task CoherentNetworkCaptureIndependentBodiesSelectionAndDisconnect(bool pvs)
    {
        await using var pair = await StartConnectedPair();
        var (client, server) = pair;
        await RunTicksSync(server, client, 10);
        EntityUid world = default, carrier = default, cargo = default, passenger = default;
        NetEntity worldNet = default, carrierNet = default, cargoNet = default, passengerNet = default;
        await server.WaitPost(() =>
        {
            server.CfgMan.SetCVar(CVars.NetPVS, pvs);
            world = server.System<SharedSpatialSystem>().CreateWorld();
            var physics = server.System<SharedSpatialPhysicsSystem>();
            physics.EnablePhysics(world, new SpatialPhysicsSettings(Vector3.Zero, 1, 8));
            carrier = server.EntMan.SpawnAttachedTo(null, world, RigidTransform3.Identity);
            cargo = server.EntMan.SpawnAttachedTo(null, carrier, new RigidTransform3(Vector3.UnitX, Quaternion.Identity));
            passenger = server.EntMan.CreateEntityUninitialized(null, carrier,
                new RigidTransform3(new Vector3(0, 2, 1), Quaternion.Identity));
            var body = server.EntMan.AddComponent<SpatialBodyComponent>(passenger);
            physics.ConfigureInitialBody((passenger, body), new SpatialBodyDefinition(SpatialMotionOwner.DynamicBody,
                new[] { new SpatialColliderDefinition(new SpatialBoxShape(new Vector3(0.1f)), RigidTransform3.Identity) },
                collisionMask: 0));
            server.EntMan.InitializeAndStartEntity(passenger);
            worldNet = server.EntMan.GetNetEntity(world); carrierNet = server.EntMan.GetNetEntity(carrier);
            cargoNet = server.EntMan.GetNetEntity(cargo); passengerNet = server.EntMan.GetNetEntity(passenger);
            var session = server.PlayerMan.Sessions.First();
            server.PlayerMan.SetAttachedEntity(session, cargo); server.PlayerMan.JoinGame(session);
        });
        await RunTicksSync(server, client, 20);
        SpatialPoseHistory history = default!, bounded = default!;
        EntityUid clientWorld = default, clientCargo = default, clientPassenger = default, clientCarrier = default, local = default;
        var captured = new List<(uint Tick, RigidTransform3 Pose)>();
        Action<GameStateAppliedArgs> observe = args =>
        {
            if (history.TrySample(clientCargo, args.AppliedState.ToSequence.Value, out var pose))
                captured.Add((args.AppliedState.ToSequence.Value, pose));
        };
        await client.WaitAssertion(() =>
        {
            clientWorld = client.EntMan.GetEntity(worldNet); clientCarrier = client.EntMan.GetEntity(carrierNet);
            clientCargo = client.EntMan.GetEntity(cargoNet); clientPassenger = client.EntMan.GetEntity(passengerNet);
            var system = client.System<SpatialPoseHistorySystem>();
            local = client.EntMan.SpawnAttachedTo(null, clientWorld, RigidTransform3.Identity);
            history = system.Track(clientWorld, new[] { clientCargo, clientPassenger, local }, new SpatialPoseTimelineLimits(64, 8, 512));
            bounded = system.Track(clientWorld, new[] { clientCargo }, new SpatialPoseTimelineLimits(2, 2, 4));
            Assert.That(history.TrySample(clientCargo, double.MaxValue, out _), Is.False, "No capture from live predicted transforms.");
            client.ResolveDependency<IClientGameStateManager>().GameStateApplied += observe;
        });
        await RunTicksSync(server, client, 5);
        await server.WaitPost(() => server.System<SharedTransformSystem>().SetLocalPose(carrier,
            new RigidTransform3(new Vector3(3, 4, 5), Quaternion.CreateFromAxisAngle(Vector3.UnitZ, MathF.PI / 2))));
        await RunTicksSync(server, client, 10);
        await client.WaitAssertion(() =>
        {
            var upper = captured.FindIndex(sample => sample.Pose.Position.Z > 4);
            Assert.That(upper, Is.GreaterThan(0));
            var tick = (captured[upper - 1].Tick + (double) captured[upper].Tick) / 2;
            Assert.That(history.TrySample(clientCargo, tick, out var pose), Is.True);
            Assert.That(Vector3.Distance(pose.Position, new Vector3(1.5f + MathF.Sqrt(0.5f), 2 + MathF.Sqrt(0.5f), 2.5f)), Is.LessThan(1e-4));
            Assert.That(history.TrySample(clientPassenger, tick, out pose), Is.True);
            Assert.That(Vector3.Distance(pose.Position, new Vector3(0, 2, 1)), Is.LessThan(1e-4));
            Assert.That(history.TrySample(clientCarrier, tick, out _), Is.False, "A dependency is not a selected visual.");
            Assert.That(history.TrySample(local, tick, out _), Is.False, "A client-created entity has no authoritative sample.");
            Assert.That(bounded.LastInstallResult, Is.EqualTo(SpatialPoseInstallResult.BudgetExceeded));
            Assert.That(bounded.RetainedSamples, Is.Zero, "Parent closure is inside the entity budget.");
            var canonical = client.EntMan.GetComponent<TransformComponent>(clientCargo).LocalPose;
            history.TrySample(clientCargo, tick, out _);
            Assert.That(client.EntMan.GetComponent<TransformComponent>(clientCargo).LocalPose, Is.EqualTo(canonical));
            history.SetSelection(new[] { clientPassenger });
            Assert.That(history.TrySample(clientCargo, tick, out _), Is.False, "Selection revocation is immediate.");
            Assert.That(history.RetainedSamples, Is.Zero);
            client.ResolveDependency<IClientGameStateManager>().GameStateApplied -= observe;
        });
        await RunTicksSync(server, client, 5);
        await client.WaitAssertion(() => Assert.That(history.TrySample(clientPassenger, double.MaxValue, out _), Is.True));
        await server.WaitPost(() =>
        {
            var destination = server.System<SharedSpatialSystem>().CreateWorld();
            server.System<SharedSpatialPhysicsSystem>().EnablePhysics(destination, new SpatialPhysicsSettings(Vector3.Zero, 1, 8));
            server.System<SharedTransformSystem>().SetParent(passenger, destination);
        });
        await RunTicksSync(server, client, 10);
        await client.WaitAssertion(() => Assert.That(history.TrySample(clientPassenger, double.MaxValue, out _), Is.False,
            "A world transfer cannot reuse a pose from the previous world."));
        await server.WaitPost(() => server.System<SharedTransformSystem>().SetParent(passenger, carrier));
        await RunTicksSync(server, client, 10);
        await client.WaitAssertion(() => Assert.That(history.TrySample(clientPassenger, double.MaxValue, out _), Is.True));
        await server.WaitPost(() => server.EntMan.DeleteEntity(passenger));
        await RunTicksSync(server, client, 10);
        await client.WaitAssertion(() => Assert.That(history.TrySample(clientPassenger, double.MaxValue, out _), Is.False));
        await DisconnectClient(server, client);
        await client.WaitAssertion(() =>
        {
            Assert.That(history.IsDisposed, Is.True);
            Assert.That(bounded.IsDisposed, Is.True);
            Assert.That(history.RetainedSamples, Is.Zero);
            history.Dispose();
        });
    }
}
