using System;
using System.Collections.Generic;
using System.Threading.Tasks;
using Lidgren.Network;
using NUnit.Framework;
using Robust.Shared.Network;
using Robust.Shared.Serialization;

namespace Robust.UnitTesting.Shared.Networking;

[TestFixture]
public sealed class IntegrationNetworkConditionsTest : RobustIntegrationTest
{
    [Test]
    public async Task JitterPreservesReliableOrderAndQueueBudgetRejectionDoesNotRetainBytes()
    {
        await using var pair = await StartConnectedPair();
        var (client, server) = pair;
        await RunTicksSync(server, client, 10);
        var received = new List<int>();
        await server.WaitPost(() => server.NetMan.RegisterNetMessage<MsgOrderedProbe>(message => received.Add(message.Number)));
        await client.WaitAssertion(() =>
        {
            var network = client.Resolve<IntegrationNetManager>();
            network.RegisterNetMessage<MsgOrderedProbe>();
            network.ConfigureNetwork(new IntegrationNetworkConditions(TimeSpan.FromMilliseconds(50), TimeSpan.FromMilliseconds(30), MaximumQueuedBytes: 1));
            Assert.Throws<InvalidOperationException>(() => network.ClientSendMessage(new MsgOrderedProbe { Number = -1 }));
            Assert.That(network.QueuedNetworkBytes, Is.Zero);
            network.ConfigureNetwork(new IntegrationNetworkConditions(TimeSpan.FromMilliseconds(50), TimeSpan.FromMilliseconds(30)));
            for (var i = 0; i < 8; i++) network.ClientSendMessage(new MsgOrderedProbe { Number = i });
            Assert.That(network.QueuedNetworkBytes, Is.EqualTo(8 * sizeof(int)));
        });
        await client.WaitPost(() => client.Resolve<IntegrationNetManager>().AdvanceNetwork(TimeSpan.FromMilliseconds(19)));
        await server.WaitRunTicks(1);
        await server.WaitAssertion(() => Assert.That(received, Is.Empty));
        await client.WaitPost(() => client.Resolve<IntegrationNetManager>().AdvanceNetwork(TimeSpan.FromMilliseconds(61)));
        await server.WaitRunTicks(1);
        await server.WaitAssertion(() => Assert.That(received, Is.EqualTo(new[] { 0, 1, 2, 3, 4, 5, 6, 7 })));
        await client.WaitAssertion(() =>
        {
            var network = client.Resolve<IntegrationNetManager>();
            Assert.That(network.QueuedNetworkBytes, Is.Zero);
            network.ClientSendMessage(new MsgOrderedProbe { Number = 8 });
            Assert.That(network.QueuedNetworkBytes, Is.GreaterThan(0));
        });
        await DisconnectClient(server, client);
        await client.WaitAssertion(() => Assert.That(client.Resolve<IntegrationNetManager>().QueuedNetworkBytes, Is.Zero));
        await server.WaitAssertion(() => Assert.That(received.Count, Is.EqualTo(8), "Disconnect must discard undelivered packets."));
    }

    public sealed class MsgOrderedProbe : NetMessage
    {
        public int Number;
        public override MsgGroups MsgGroup => MsgGroups.EntityEvent;
        public override void WriteToBuffer(NetOutgoingMessage buffer, IRobustSerializer serializer) => buffer.Write(Number);
        public override void ReadFromBuffer(NetIncomingMessage buffer, IRobustSerializer serializer) => Number = buffer.ReadInt32();
    }
}
