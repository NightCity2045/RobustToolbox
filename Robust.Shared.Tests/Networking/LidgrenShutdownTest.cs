using System.Net;
using System.Reflection;
using Lidgren.Network;
using NUnit.Framework;

namespace Robust.Shared.Tests.Networking;

[TestFixture]
public sealed class LidgrenShutdownTest
{
    [Test]
    public void FinalHeartbeatDoesNotConsumeAcksAfterDisconnectResetsChannels()
    {
        // Drive the transport synchronously: no socket timing is needed to
        // reproduce the pending-ACK / shutdown ordering seen in the UDP tests.
        var peer = new NetPeer(new NetPeerConfiguration("shutdown-regression"));
        typeof(NetPeer).GetField("m_networkThread", BindingFlags.Instance | BindingFlags.NonPublic)!
            .SetValue(peer, Thread.CurrentThread);
        var connection = new NetConnection(peer, new IPEndPoint(IPAddress.Loopback, 12345));
        var type = NetMessageType.UserReliableOrdered1;
        var channel = new NetReliableSenderChannel(connection, 64);
        connection.m_sendChannels[(int) type - 1] = channel;
        connection.m_status = NetConnectionStatus.Connected;
        connection.m_queuedIncomingAcks.Enqueue((type, 0));

        connection.ExecuteDisconnect("Test shutdown", sendByeMessage: false);
        Assert.That(connection.m_status, Is.EqualTo(NetConnectionStatus.Disconnected));
        Assert.DoesNotThrow(() => connection.Heartbeat(NetTime.Now, 3));
        Assert.That(connection.m_queuedIncomingAcks.Count, Is.EqualTo(1),
            "Disconnected channels must never process the old connection's ACKs.");
    }
}
