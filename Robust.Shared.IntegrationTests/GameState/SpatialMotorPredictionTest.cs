using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Threading.Tasks;
using NUnit.Framework;
using Robust.Client.GameStates;
using Robust.Client.Spatial;
using Robust.Client.Timing;
using Robust.Shared;
using Robust.Shared.Containers;
using Robust.Shared.GameObjects;
using Robust.Shared.IoC;
using Robust.Shared.Maths;
using Robust.Shared.Network;
using Robust.Shared.Serialization;
using Robust.Shared.Spatial;
using Robust.Shared.Spatial.Physics;
using Robust.Shared.Timing;

namespace Robust.UnitTesting.Shared.GameState;

[TestFixture]
public sealed partial class SpatialMotorPredictionTest : RobustIntegrationTest
{
    [Test]
    public async Task PassengerPredictionUsesTheMovingCompoundSupportOnce()
    {
        await using var pair = await StartConnectedPair();
        var (client, server) = pair;
        await RunTicksSync(server, client, 10);
        NetEntity motorNet = default, deckNet = default;
        EntityUid motor = default;
        await server.WaitPost(() =>
        {
            var world = server.System<SharedSpatialSystem>().CreateWorld();
            var physics = server.System<SharedSpatialPhysicsSystem>();
            physics.EnablePhysics(world, new SpatialPhysicsSettings(-Vector3.UnitZ * 9.81f, 4, 8));
            var carrier = server.EntMan.CreateEntityUninitialized(null, world,
                new RigidTransform3(new Vector3(0, 0, -0.6f), Quaternion.Identity));
            var rootBody = server.EntMan.AddComponent<SpatialBodyComponent>(carrier);
            physics.ConfigureInitialBody((carrier, rootBody), new SpatialBodyDefinition(SpatialMotionOwner.DynamicBody,
                new[] { new SpatialColliderDefinition(new SpatialBoxShape(new Vector3(0.2f)), RigidTransform3.Identity) }, mass: 1000000),
                new Vector3(1, 0, 0.2f), Vector3.UnitZ * 0.3f);
            server.EntMan.InitializeAndStartEntity(carrier);
            server.System<PredictionInputSystem>().Carrier = carrier;
            var deck = server.EntMan.CreateEntityUninitialized(null, carrier,
                new RigidTransform3(new Vector3(0, 0, 0.6f), Quaternion.Identity));
            var layers = server.EntMan.AddComponent<SpatialFrameComponent>(deck);
            var layer = new LayerId(1);
            server.System<SharedSpatialSystem>().ConfigureInitialLayers((deck, layers),
                new Dictionary<LayerId, LayerDefinition> { [layer] = new(0, "Deck") });
            var geometry = server.EntMan.AddComponent<SpatialGeometryComponent>(deck);
            server.System<SharedGeometrySystem>().ConfigureInitialGeometry((deck, geometry), Array.Empty<BoundarySegment>(),
                new[] { new SlabSurface(new SpatialSurfaceReference(deck, new SurfaceId(1)), layer, new Box2(-5, -5, 5, 5), 0, 0.2f) });
            var deckBody = server.EntMan.AddComponent<SpatialBodyComponent>(deck);
            physics.ConfigureInitialBody((deck, deckBody), new SpatialBodyDefinition(SpatialMotionOwner.AnchoredCompoundChild,
                useGeometry: true, mass: 1000000, collisionLayer: 2));
            server.EntMan.InitializeAndStartEntity(deck);
            motor = server.EntMan.CreateEntityUninitialized(null, carrier,
                new RigidTransform3(new Vector3(2, 0, 1.4f), Quaternion.Identity));
            var controller = server.EntMan.AddComponent<SpatialMotorComponent>(motor);
            physics.ConfigureInitialMotor((motor, controller), SpatialMotorSettings.Default);
            var body = server.EntMan.AddComponent<SpatialBodyComponent>(motor);
            physics.ConfigureInitialBody((motor, body), new SpatialBodyDefinition(SpatialMotionOwner.Motor,
                new[] { new SpatialColliderDefinition(new SpatialCapsuleShape(0.3f, 1), RigidTransform3.Identity) },
                mass: 80, collisionMask: 2));
            server.EntMan.InitializeAndStartEntity(motor);
            motorNet = server.EntMan.GetNetEntity(motor); deckNet = server.EntMan.GetNetEntity(deck);
            var session = server.PlayerMan.Sessions.First();
            server.PlayerMan.SetAttachedEntity(session, motor); server.PlayerMan.JoinGame(session);
        });
        await RunTicksSync(server, client, 40);
        SpatialMotorPrediction prediction = default!;
        EntityUid localMotor = default;
        await client.WaitPost(() =>
        {
            localMotor = client.EntMan.GetEntity(motorNet);
            client.CfgMan.SetCVar(CVars.NetPredictTickBias, 6);
            prediction = client.System<SpatialMotorPredictionSystem>().PredictMotor(localMotor,
                new Box3(new Vector3(-2, -4, -2), new Vector3(8, 4, 4)), new SpatialCollisionViewLimits(1000, 1000, 1000), 30);
        });
        await RunTicksSync(server, client, 30);
        Vector3 serverPosition = default;
        await server.WaitAssertion(() =>
        {
            serverPosition = server.System<SharedTransformSystem>().GetWorldPose(motor).Position;
            Assert.That(server.EntMan.GetComponent<SpatialMotorComponent>(motor).Support, Is.Not.Null);
        });
        await client.WaitAssertion(() =>
        {
            Assert.That(prediction.Status, Is.EqualTo(SpatialMotorPredictionStatus.Active));
            Assert.That(prediction.PredictedSteps, Is.GreaterThan(30));
            var controller = client.EntMan.GetComponent<SpatialMotorComponent>(localMotor);
            Assert.That(controller.Support, Is.Not.Null);
            var support = controller.Support!.Value;
            Assert.That(support.Source.Surface!.Value.OwnerFrame, Is.EqualTo(client.EntMan.GetEntity(deckNet)));
            Assert.That(support.Velocity.Y, Is.GreaterThan(0.3f));
            Assert.That(support.Velocity.Z, Is.EqualTo(0.2f).Within(0.01f));
            var position = client.System<SharedTransformSystem>().GetWorldPose(localMotor).Position;
            Assert.That(Vector3.Distance(position, serverPosition), Is.LessThan(0.45f));
            Assert.That(position.Z - serverPosition.Z, Is.LessThan(0.1f));
            var localCarrier = client.EntMan.GetComponent<TransformComponent>(localMotor).ParentUid;
            Assert.That(client.System<SharedTransformSystem>().GetWorldPose(localCarrier),
                Is.EqualTo(prediction.Capture!.Entities[localCarrier].Pose), "Remote proxy motion must not write canonical parent poses.");
            prediction.Dispose();
        });
    }

    [TestCase(true, 0)]
    [TestCase(false, 0)]
    [TestCase(true, 100)]
    [TestCase(true, 200)]
    [TestCase(false, 200)]
    public async Task InputsReplayAgainstKnownCollisionAndServerRejectionRestoresMotion(bool pvs, int roundTripMilliseconds)
    {
        await using var pair = await StartConnectedPair();
        var (client, server) = pair;
        var impaired = false;
        if (roundTripMilliseconds != 0)
            await server.WaitPost(() => server.CfgMan.SetCVar(CVars.NetTickrate, 60));
        async Task Step(int ticks)
        {
            for (var i = 0; i < ticks * (roundTripMilliseconds == 0 ? 1 : 2); i++)
            {
                if (impaired)
                {
                    var elapsed = TimeSpan.FromSeconds(1d / 60);
                    await server.WaitPost(() => server.Resolve<IntegrationNetManager>().AdvanceNetwork(elapsed));
                    await client.WaitPost(() => client.Resolve<IntegrationNetManager>().AdvanceNetwork(elapsed));
                }
                await RunTicksSync(server, client, 1);
            }
        }
        await Step(10);
        NetEntity motorNet = default;
        EntityUid motor = default;
        await server.WaitPost(() =>
        {
            server.CfgMan.SetCVar(CVars.NetPVS, pvs);
            var world = server.System<SharedSpatialSystem>().CreateWorld();
            var physics = server.System<SharedSpatialPhysicsSystem>();
            physics.EnablePhysics(world, new SpatialPhysicsSettings(-Vector3.UnitZ * 9.81f, 4, 8));
            var frame = server.EntMan.CreateEntityUninitialized(null, world, RigidTransform3.Identity);
            var layers = server.EntMan.AddComponent<SpatialFrameComponent>(frame);
            var layer = new LayerId(1);
            server.System<SharedSpatialSystem>().ConfigureInitialLayers((frame, layers),
                new Dictionary<LayerId, LayerDefinition> { [layer] = new(0, "Deck") });
            var geometry = server.EntMan.AddComponent<SpatialGeometryComponent>(frame);
            server.System<SharedGeometrySystem>().ConfigureInitialGeometry((frame, geometry), Array.Empty<BoundarySegment>(),
                new[] { new SlabSurface(new SpatialSurfaceReference(frame, new SurfaceId(1)), layer, new Box2(-8, -8, 8, 8), 0, 0.2f) });
            server.EntMan.InitializeAndStartEntity(frame);
            var wall = server.EntMan.CreateEntityUninitialized(null, world, new RigidTransform3(new Vector3(2, 0, 2), Quaternion.Identity));
            var wallBody = server.EntMan.AddComponent<SpatialBodyComponent>(wall);
            physics.ConfigureInitialBody((wall, wallBody), new SpatialBodyDefinition(SpatialMotionOwner.Static,
                new[] { new SpatialColliderDefinition(new SpatialBoxShape(new Vector3(0.1f, 8, 4)), RigidTransform3.Identity) }));
            server.EntMan.InitializeAndStartEntity(wall);
            motor = server.EntMan.CreateEntityUninitialized(null, world, new RigidTransform3(new Vector3(0, 0, 0.8f), Quaternion.Identity));
            var controller = server.EntMan.AddComponent<SpatialMotorComponent>(motor);
            physics.ConfigureInitialMotor((motor, controller), SpatialMotorSettings.Default);
            var body = server.EntMan.AddComponent<SpatialBodyComponent>(motor);
            physics.ConfigureInitialBody((motor, body), new SpatialBodyDefinition(SpatialMotionOwner.Motor,
                new[] { new SpatialColliderDefinition(new SpatialCapsuleShape(0.3f, 1), RigidTransform3.Identity) }, mass: 80));
            server.EntMan.InitializeAndStartEntity(motor);
            motorNet = server.EntMan.GetNetEntity(motor);
            var session = server.PlayerMan.Sessions.First();
            server.PlayerMan.SetAttachedEntity(session, motor);
            server.PlayerMan.JoinGame(session);
        });
        if (roundTripMilliseconds != 0)
        {
            var conditions = new IntegrationNetworkConditions(TimeSpan.FromMilliseconds(roundTripMilliseconds / 2d),
                TimeSpan.FromMilliseconds(10), DropEveryNthUnreliable: 5);
            await server.WaitPost(() => server.Resolve<IntegrationNetManager>().ConfigureNetwork(conditions));
            await client.WaitPost(() => client.Resolve<IntegrationNetManager>().ConfigureNetwork(conditions));
            impaired = true;
        }
        await Step(40);
        SpatialMotorPrediction prediction = default!;
        EntityUid localMotor = default;
        await client.WaitAssertion(() =>
        {
            client.CfgMan.SetCVar(CVars.NetPredictTickBias, 8);
            localMotor = client.EntMan.GetEntity(motorNet);
            prediction = client.System<SpatialMotorPredictionSystem>().PredictMotor(localMotor,
                new Box3(new Vector3(-4), new Vector3(4)), new SpatialCollisionViewLimits(1000, 1000, 1000), 30);
        });
        await Step(20);
        await client.WaitAssertion(() =>
        {
            Assert.That(prediction.PredictedSteps, Is.GreaterThan(0));
            Assert.That(prediction.Status, Is.EqualTo(SpatialMotorPredictionStatus.Active));
            Assert.That(client.System<PredictionInputSystem>().LandingEvents, Is.Zero);
        });

        await server.WaitPost(() => server.System<PredictionInputSystem>().Reject = true);
        await client.WaitPost(() => client.EntMan.RaisePredictiveEvent(new MotorInput(motorNet, Vector3.UnitX * 4)));
        var peak = 0f;
        for (var i = 0; i < 30; i++)
        {
            await Step(1);
            await client.WaitPost(() => peak = Math.Max(peak, client.System<SharedTransformSystem>().GetWorldPose(localMotor).Position.X));
        }
        await client.WaitAssertion(() =>
        {
            Assert.That(peak, Is.GreaterThan(0.01f), "Unacknowledged input must actually predict movement.");
            Assert.That(client.System<SharedTransformSystem>().GetWorldPose(localMotor).Position.X, Is.EqualTo(0).Within(0.001f));
            Assert.That(client.System<PredictionInputSystem>().Inputs, Is.GreaterThan(1), "The existing event queue must replay the input.");
        });
        await server.WaitAssertion(() =>
        {
            Assert.That(server.System<PredictionInputSystem>().Inputs, Is.EqualTo(1));
            Assert.That(server.System<SharedTransformSystem>().GetWorldPose(motor).Position.X, Is.EqualTo(0).Within(0.001f));
            server.System<PredictionInputSystem>().Reject = false;
        });
        await client.WaitPost(() => client.EntMan.RaisePredictiveEvent(new MotorInput(motorNet, Vector3.UnitX * 4)));
        await Step(100);
        await client.WaitAssertion(() =>
        {
            var pose = client.System<SharedTransformSystem>().GetWorldPose(localMotor);
            Assert.That(pose.Position.X, Is.InRange(1.5f, 1.68f));
            Assert.That(pose.Position.Z, Is.EqualTo(0.8f).Within(0.02f));
            Assert.That(prediction.Status, Is.EqualTo(SpatialMotorPredictionStatus.Active));
            Assert.That(client.System<PredictionInputSystem>().LandingEvents, Is.Zero);
        });
        await server.WaitAssertion(() => Assert.That(server.System<PredictionInputSystem>().Inputs, Is.EqualTo(2)));

        await client.WaitPost(() => client.EntMan.RaisePredictiveEvent(new MotorInput(motorNet, Vector3.Zero, resize: true)));
        await Step(25);
        await client.WaitAssertion(() =>
        {
            var body = client.EntMan.GetComponent<SpatialBodyComponent>(localMotor);
            Assert.That(body.DefinitionRevision, Is.EqualTo(1));
            Assert.That(((SpatialCapsuleShape) body.Definition.Colliders[0].Shape).Length, Is.EqualTo(0.5f));
            Assert.That(client.System<SharedTransformSystem>().GetWorldPose(localMotor).Position.Z, Is.EqualTo(0.8f).Within(0.02f));
            Assert.That(client.System<PredictionInputSystem>().PredictedResizes, Is.GreaterThan(0));
        });
        await client.WaitPost(() => client.EntMan.RaisePredictiveEvent(new MotorInput(motorNet, Vector3.Zero, jump: true)));
        var jumpPeak = 0f;
        for (var i = 0; i < 45; i++)
        {
            await Step(1);
            await client.WaitPost(() => jumpPeak = Math.Max(jumpPeak, client.System<SharedTransformSystem>().GetWorldPose(localMotor).Position.Z));
        }
        await client.WaitAssertion(() =>
        {
            Assert.That(jumpPeak, Is.GreaterThan(1.7f));
            Assert.That(client.System<SharedTransformSystem>().GetWorldPose(localMotor).Position.Z, Is.EqualTo(0.8f).Within(0.03f));
            Assert.That(client.System<PredictionInputSystem>().LandingEvents, Is.Zero);
        });
        await server.WaitAssertion(() =>
        {
            Assert.That(server.System<PredictionInputSystem>().Inputs, Is.EqualTo(4));
            Assert.That(server.System<PredictionInputSystem>().LandingEvents, Is.GreaterThan(0));
            Assert.That(server.EntMan.GetComponent<SpatialBodyComponent>(motor).DefinitionRevision, Is.EqualTo(1));
            server.System<PredictionInputSystem>().Reject = true;
        });

        await client.WaitPost(() => client.EntMan.RaisePredictiveEvent(new MotorInput(motorNet, Vector3.Zero,
            impulse: Vector3.UnitY * 40000)));
        var blocked = false;
        var unsafeDistance = 0f;
        for (var i = 0; i < 20; i++)
        {
            await Step(1);
            await client.WaitPost(() =>
            {
                blocked |= prediction.Status == SpatialMotorPredictionStatus.OutsideCoverage;
                unsafeDistance = Math.Max(unsafeDistance, MathF.Abs(client.System<SharedTransformSystem>().GetWorldPose(localMotor).Position.Y));
            });
        }
        await client.WaitAssertion(() =>
        {
            Assert.That(blocked, Is.True, "A speculative impulse must reach the actual coverage check.");
            Assert.That(unsafeDistance, Is.LessThan(0.001f), "No partial pose from an uncertified step may be committed.");
        });
#if DEBUG
        await server.WaitPost(() => server.System<PredictionInputSystem>().Reject = false);
        await client.WaitPost(() =>
        {
            ((ClientGameStateManager) client.Resolve<IClientGameStateManager>()).DropStates = true;
            client.EntMan.RaisePredictiveEvent(new MotorInput(motorNet, -Vector3.UnitX * 2));
        });
        await Step(20);
        await client.WaitPost(() => ((ClientGameStateManager) client.Resolve<IClientGameStateManager>()).DropStates = false);
        await Step(20);
        await client.WaitPost(() => client.EntMan.RaisePredictiveEvent(new MotorInput(motorNet, Vector3.Zero)));
        await Step(15);
        Vector3 afterOutage = default;
        await server.WaitAssertion(() =>
        {
            Assert.That(server.System<PredictionInputSystem>().Inputs, Is.EqualTo(7));
            afterOutage = server.System<SharedTransformSystem>().GetWorldPose(motor).Position;
        });
        await client.WaitAssertion(() =>
        {
            Assert.That(prediction.Status, Is.EqualTo(SpatialMotorPredictionStatus.Active));
            Assert.That(Vector3.Distance(client.System<SharedTransformSystem>().GetWorldPose(localMotor).Position, afterOutage), Is.LessThan(0.05f));
        });
#endif

        await client.WaitAssertion(() =>
        {
            prediction.SetBounds(new Box3(new Vector3(-1000), new Vector3(1000)));
        });
        await Step(8);
        await client.WaitAssertion(() => Assert.That(prediction.Status, Is.EqualTo(SpatialMotorPredictionStatus.GeometryUnavailable)));
        await client.WaitPost(() => prediction.SetBounds(new Box3(new Vector3(-4), new Vector3(4))));
        await Step(8);
        await client.WaitAssertion(() => Assert.That(prediction.Status, Is.EqualTo(SpatialMotorPredictionStatus.Active)));
        await client.WaitPost(() =>
        {
            prediction.Dispose();
            prediction = client.System<SpatialMotorPredictionSystem>().PredictMotor(localMotor,
                new Box3(new Vector3(-4), new Vector3(4)), new SpatialCollisionViewLimits(1000, 1000, 1000), 1);
        });
        await Step(10);
        await client.WaitAssertion(() =>
        {
            Assert.That(prediction.Status, Is.EqualTo(SpatialMotorPredictionStatus.PredictionLimit));
            var timing = client.Resolve<IClientGameTiming>();
            Assert.That(prediction.LastPredictedTick!.Value.Value - timing.LastRealTick.Value, Is.EqualTo(1));
        });
        await client.WaitPost(() =>
        {
            prediction.Dispose();
            prediction = client.System<SpatialMotorPredictionSystem>().PredictMotor(localMotor,
                new Box3(new Vector3(-4), new Vector3(4)), new SpatialCollisionViewLimits(1000, 1000, 1000), 30);
        });
        await Step(10);
        ContainerSlot container = default!;
        await server.WaitAssertion(() =>
        {
            var world = server.EntMan.GetComponent<TransformComponent>(motor).MapUid!.Value;
            var holder = server.EntMan.SpawnAttachedTo(null, world, RigidTransform3.Identity);
            var containers = server.System<SharedContainerSystem>();
            container = containers.EnsureContainer<ContainerSlot>(holder, "prediction-test");
            Assert.That(containers.Insert(motor, container), Is.True);
        });
        await Step(10);
        ulong dormantSteps = 0;
        await client.WaitAssertion(() =>
        {
            Assert.That(prediction.Status, Is.EqualTo(SpatialMotorPredictionStatus.InactiveBody));
            dormantSteps = prediction.PredictedSteps;
        });
        await Step(10);
        await client.WaitAssertion(() => Assert.That(prediction.PredictedSteps, Is.EqualTo(dormantSteps)));
        await server.WaitAssertion(() =>
        {
            Assert.That(server.System<SharedContainerSystem>().Remove(motor, container), Is.True);
            server.System<SharedTransformSystem>().SetWorldPose(motor, new RigidTransform3(new Vector3(0, 0, 0.8f), Quaternion.Identity));
            server.System<SharedSpatialPhysicsSystem>().SetVelocity((motor, server.EntMan.GetComponent<SpatialBodyComponent>(motor)), Vector3.Zero, Vector3.Zero);
        });
        await Step(15);
        await client.WaitAssertion(() => Assert.That(prediction.Status, Is.EqualTo(SpatialMotorPredictionStatus.Active)));
        if (impaired)
        {
            void VerifyTransport(IntegrationNetManager network, bool expectLoss)
            {
                Assert.That(network.DeliveredDelayedPackets, Is.GreaterThan(50));
                if (expectLoss) Assert.That(network.DroppedNetworkPackets, Is.GreaterThan(0));
                Assert.That(network.MinimumDeliveredDelay!.Value.TotalMilliseconds, Is.GreaterThanOrEqualTo(roundTripMilliseconds / 2d - 10));
                Assert.That(network.MaximumDeliveredDelay.TotalMilliseconds, Is.LessThanOrEqualTo(roundTripMilliseconds / 2d + 10 + 1000d / 60 + 0.001));
                TestContext.Out.WriteLine($"RTT profile {roundTripMilliseconds}ms, one-way observed " +
                    $"{network.MinimumDeliveredDelay.Value.TotalMilliseconds:F3}..{network.MaximumDeliveredDelay.TotalMilliseconds:F3}ms, " +
                    $"delivered {network.DeliveredDelayedPackets}, dropped {network.DroppedNetworkPackets}");
            }
            await server.WaitAssertion(() => VerifyTransport(server.Resolve<IntegrationNetManager>(), true));
            await client.WaitAssertion(() => VerifyTransport(client.Resolve<IntegrationNetManager>(), false));
        }
        await DisconnectClient(server, client);
        await client.WaitAssertion(() => Assert.That(prediction.IsDisposed, Is.True));
        await server.WaitAssertion(() => Assert.That(server.Resolve<IntegrationNetManager>().QueuedNetworkBytes, Is.Zero));
        await client.WaitAssertion(() => Assert.That(client.Resolve<IntegrationNetManager>().QueuedNetworkBytes, Is.Zero));
    }

    public sealed partial class PredictionInputSystem : EntitySystem
    {
        [Dependency] private INetManager _net = default!;
        [Dependency] private IGameTiming _timing = default!;
        [Dependency] private SharedSpatialPhysicsSystem _physics = default!;
        public bool Reject;
        public int Inputs;
        public int LandingEvents;
        public int PredictedResizes;
        public EntityUid? Carrier;
        public override void Initialize()
        {
            base.Initialize();
            UpdatesBefore.Add(typeof(SharedSpatialPhysicsSystem));
            SubscribeAllEvent<MotorInput>(OnInput);
            SubscribeLocalEvent<SpatialMotorComponent, SpatialMotorLanded>(OnLanded);
        }
        private void OnLanded(EntityUid uid, SpatialMotorComponent motor, ref SpatialMotorLanded args) => LandingEvents++;
        public override void Update(float frameTime)
        {
            if (_net.IsServer && Carrier is { } carrier)
                _physics.SetVelocity((carrier, Comp<SpatialBodyComponent>(carrier)),
                    new Vector3(1, 0, 0.2f + 9.81f * frameTime), Vector3.UnitZ * 0.3f);
        }
        private void OnInput(MotorInput input, EntitySessionEventArgs args)
        {
            if (_net.IsClient && !_timing.InPrediction) return;
            Inputs++;
            if (Reject || !TryGetEntity(input.Entity, out var uid) || args.SenderSession.AttachedEntity != uid) return;
            _physics.SetMotorIntent((uid.Value, Comp<SpatialMotorComponent>(uid.Value)), new SpatialMotorIntent(input.Velocity, SpatialMotorMode.Walking));
            if (input.Resize)
            {
                var resized = _physics.TryResizeMotor((uid.Value, Comp<SpatialMotorComponent>(uid.Value)), 0.25f, 0.5f, 0);
                if (_net.IsServer) Assert.That(resized, Is.True);
                else if (resized) PredictedResizes++;
            }
            if (input.Jump)
                _physics.ApplyImpulse((uid.Value, Comp<SpatialBodyComponent>(uid.Value)), Vector3.UnitZ * 400,
                    EntityManager.System<SharedTransformSystem>().GetWorldPose(uid.Value).Position);
            if (input.Impulse != Vector3.Zero)
                _physics.ApplyImpulse((uid.Value, Comp<SpatialBodyComponent>(uid.Value)), input.Impulse,
                    EntityManager.System<SharedTransformSystem>().GetWorldPose(uid.Value).Position);
        }
    }

    [Serializable, NetSerializable]
    public sealed class MotorInput(NetEntity entity, Vector3 velocity, bool resize = false, bool jump = false, Vector3 impulse = default) : EntityEventArgs
    {
        public NetEntity Entity { get; } = entity;
        public Vector3 Velocity { get; } = velocity;
        public bool Resize { get; } = resize;
        public bool Jump { get; } = jump;
        public Vector3 Impulse { get; } = impulse;
    }
}
