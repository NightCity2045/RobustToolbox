using System.Diagnostics;
using System.Globalization;
using System.Net;
using System.Net.Sockets;
using Lidgren.Network;
using NUnit.Framework;
using Robust.Client;
using Robust.Shared;
using Robust.Shared.IoC;
using Robust.Shared.Log;
using Robust.Shared.Network;
using Robust.Shared.Serialization;
using Robust.Shared.Spatial;

namespace Robust.UnitTesting.Shared.Networking;

/// <summary>Uses the actual socket handshake, not IntegrationNetManager's shortcut.</summary>
[TestFixture, NonParallelizable]
public sealed class SpatialHandshakeTest : RobustIntegrationTest
{
    private static ClientIntegrationOptions SocketClient(bool realStrings, bool expectedFailure = false)
    {
        var options = new ClientIntegrationOptions
        {
            Pool = false,
            // NetManager deliberately logs a rejected handshake as an error.
            // The negative tests assert its exact public failure reason instead.
            FailureLogLevel = expectedFailure ? LogLevel.Fatal : LogLevel.Error,
            InitIoC = () =>
            {
                IoCManager.Register<INetManager, NetManager>(overwrite: true);
                IoCManager.Register<IClientNetManager, NetManager>(overwrite: true);
                if (realStrings)
                    IoCManager.Register<IRobustMappedStringSerializer, RobustMappedStringSerializer>(overwrite: true);
            }
        };
        options.CVarOverrides[CVars.NetEncrypt.Name] = "false";
        options.CVarOverrides[CVars.PlayerName.Name] = "SpatialProtocolTest";
        return options;
    }

    [TestCase(0, "ordinary Robust 295")]
    [TestCase(1, "protocol revision")]
    [TestCase(2, "schema/physics fingerprint")]
    public async Task ServerRejectsIncompatibleClientBeforeCreatingAChannel(int variant, string reason)
    {
        int port;
        using (var reservation = new Socket(AddressFamily.InterNetwork, SocketType.Dgram, ProtocolType.Udp))
        {
            reservation.Bind(new IPEndPoint(IPAddress.Loopback, 0));
            port = ((IPEndPoint) reservation.LocalEndPoint!).Port;
        }
        var options = new ServerIntegrationOptions
        {
            Pool = false,
            InitIoC = () =>
            {
                IoCManager.Register<INetManager, NetManager>(overwrite: true);
                IoCManager.Register<IServerNetManager, NetManager>(overwrite: true);
            }
        };
        options.CVarOverrides[CVars.NetPort.Name] = port.ToString(CultureInfo.InvariantCulture);
        options.CVarOverrides[CVars.NetBindTo.Name] = "127.0.0.1";
        using var server = StartServer(options);
        await server.WaitIdleAsync();
        var peer = new NetClient(new NetPeerConfiguration(CVars.NetLidgrenAppIdentifier.DefaultValue));
        peer.Start();
        peer.Connect(new IPEndPoint(IPAddress.Loopback, port));
        var sent = false;
        string? rejection = null;
        var deadline = Stopwatch.StartNew();
        try
        {
            while (deadline.Elapsed < TimeSpan.FromSeconds(10) && rejection == null)
            {
                await server.WaitRunTicks(1);
                while (peer.ReadMessage() is { } message)
                {
                    if (message.MessageType != NetIncomingMessageType.StatusChanged)
                        continue;
                    var status = (NetConnectionStatus) message.ReadByte();
                    var text = message.ReadString();
                    if (status == NetConnectionStatus.Connected && !sent)
                    {
                        var login = peer.CreateMessage();
                        login.Write("SpatialProtocolTest");
                        login.Write(false); // CanAuth
                        login.Write(false); // NeedPubKey
                        login.Write(false); // Encrypt
                        if (variant != 0)
                        {
                            var stamp = variant == 1
                                ? SpatialProtocol.Current with { Revision = SpatialProtocol.Revision + 1 }
                                : SpatialProtocol.Current with { Fingerprint = new string('0', 64) };
                            SpatialProtocol.Write(login, stamp);
                        }
                        peer.SendMessage(login, NetDeliveryMethod.ReliableOrdered);
                        sent = true;
                    }
                    if (status == NetConnectionStatus.Disconnected)
                        rejection = text;
                }
                await Task.Delay(5);
            }
            Assert.That(sent, Is.True, "Raw Lidgren connection was not established.");
            Assert.That(rejection, Does.Contain(reason));
            await server.WaitAssertion(() => Assert.That(server.NetMan.ChannelCount, Is.Zero));
        }
        finally { peer.Shutdown("Handshake test complete."); }
    }

    [TestCase(false)]
    [TestCase(true)]
    public async Task ClientRejectsLegacySuccessOrAuthenticationChallengeBeforeCreatingAChannel(bool challenge)
    {
        var peer = new NetServer(new NetPeerConfiguration(CVars.NetLidgrenAppIdentifier.DefaultValue)
        {
            LocalAddress = IPAddress.Loopback,
            Port = 0
        });
        peer.Start();
        using var client = StartClient(SocketClient(false, expectedFailure: true));
        await client.WaitIdleAsync();
        var failure = new TaskCompletionSource<string>(TaskCreationOptions.RunContinuationsAsynchronously);
        await client.WaitPost(() =>
        {
            var network = client.Resolve<IClientNetManager>();
            network.ConnectFailed += (_, ev) => failure.TrySetResult(ev.Reason);
            client.Resolve<IBaseClient>().ConnectToServer(new DnsEndPoint("127.0.0.1", peer.Port));
        });
        var sent = false;
        var deadline = Stopwatch.StartNew();
        try
        {
            while (!failure.Task.IsCompleted && deadline.Elapsed < TimeSpan.FromSeconds(10))
            {
                await client.WaitRunTicks(1);
                while (peer.ReadMessage() is { } message)
                {
                    if (message.MessageType != NetIncomingMessageType.Data || sent || message.SenderConnection is not { } connection)
                        continue;
                    var reply = peer.CreateMessage();
                    reply.Write(!challenge);
                    reply.WritePadBits();
                    if (challenge)
                    {
                        // Intentionally invalid keys: protocol rejection must happen
                        // before the client attempts authentication or cryptography.
                        reply.WriteVariableInt32(1);
                        reply.Write(new byte[1]);
                        reply.WriteVariableInt32(1);
                        reply.Write(new byte[1]);
                        reply.Write(false);
                    }
                    else
                    {
                        reply.Write("LegacyServer");
                        reply.Write(Guid.NewGuid());
                        reply.Write("");
                        reply.Write((byte) LoginType.GuestAssigned);
                        reply.Write(new byte[100]);
                    }
                    peer.SendMessage(reply, connection, NetDeliveryMethod.ReliableOrdered);
                    sent = true;
                }
                await Task.Delay(5);
            }
            Assert.That(sent, Is.True);
            Assert.That(failure.Task.IsCompleted, Is.True, "The client did not reject a legacy server.");
            Assert.That(await failure.Task, Does.Contain("ordinary Robust 295"));
            await client.WaitAssertion(() => Assert.That(client.NetMan.ChannelCount, Is.Zero));
        }
        finally { peer.Shutdown("Handshake test complete."); }
    }

    [Test]
    public async Task MatchingPairCompletesActualSocketAndSerializerHandshakes()
    {
        int port;
        using (var reservation = new Socket(AddressFamily.InterNetwork, SocketType.Dgram, ProtocolType.Udp))
        {
            reservation.Bind(new IPEndPoint(IPAddress.Loopback, 0));
            port = ((IPEndPoint) reservation.LocalEndPoint!).Port;
        }
        var options = new ServerIntegrationOptions
        {
            Pool = false,
            InitIoC = () =>
            {
                IoCManager.Register<INetManager, NetManager>(overwrite: true);
                IoCManager.Register<IServerNetManager, NetManager>(overwrite: true);
                IoCManager.Register<IRobustMappedStringSerializer, RobustMappedStringSerializer>(overwrite: true);
            }
        };
        options.CVarOverrides[CVars.NetPort.Name] = port.ToString(CultureInfo.InvariantCulture);
        options.CVarOverrides[CVars.NetBindTo.Name] = "127.0.0.1";
        options.CVarOverrides[CVars.AuthMode.Name] = ((int) AuthMode.Disabled).ToString(CultureInfo.InvariantCulture);
        using var server = StartServer(options);
        using var client = StartClient(SocketClient(true));
        await Task.WhenAll(server.WaitIdleAsync(), client.WaitIdleAsync());
        var serverConnected = new TaskCompletionSource<bool>(TaskCreationOptions.RunContinuationsAsynchronously);
        var clientConnected = new TaskCompletionSource<bool>(TaskCreationOptions.RunContinuationsAsynchronously);
        await server.WaitPost(() => server.NetMan.Connected += (_, _) => serverConnected.TrySetResult(true));
        await client.WaitPost(() =>
        {
            client.NetMan.Connected += (_, _) => clientConnected.TrySetResult(true);
            client.Resolve<IBaseClient>().ConnectToServer(new DnsEndPoint("127.0.0.1", port));
        });
        var deadline = Stopwatch.StartNew();
        while ((!serverConnected.Task.IsCompleted || !clientConnected.Task.IsCompleted) && deadline.Elapsed < TimeSpan.FromSeconds(20))
        {
            await RunTicksSync(server, client, 1);
            await Task.Delay(5);
        }
        Assert.That(serverConnected.Task.IsCompleted, Is.True, "Server handshake did not complete.");
        Assert.That(clientConnected.Task.IsCompleted, Is.True, "Client handshake did not complete.");
        await client.WaitPost(() => client.Resolve<IBaseClient>().DisconnectFromServer("Handshake test complete."));
        await RunTicksSync(server, client, 5);
    }
}
