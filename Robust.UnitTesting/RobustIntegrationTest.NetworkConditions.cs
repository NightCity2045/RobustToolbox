using System;
using System.Buffers;
using System.Collections.Generic;
using Lidgren.Network;
using Robust.Shared.Network;

namespace Robust.UnitTesting;

public partial class RobustIntegrationTest
{
    /// <summary>
    /// Deterministic transport conditions on serialized test packets. Reliable
    /// packets remain reliable and ordered channels retain their order; loss is
    /// applied to unreliable datagrams. The test advances a shared virtual clock,
    /// independent of client prediction rewinds and wall-clock machine speed.
    /// </summary>
    internal sealed record IntegrationNetworkConditions(TimeSpan OneWayDelay, TimeSpan Jitter,
        int DropEveryNthUnreliable = 0, int MaximumQueuedBytes = 16 * 1024 * 1024)
    {
        public void Validate()
        {
            if (OneWayDelay < TimeSpan.Zero || Jitter < TimeSpan.Zero || Jitter > OneWayDelay
                || OneWayDelay.TotalMilliseconds * 2 > short.MaxValue || DropEveryNthUnreliable < 0 || MaximumQueuedBytes <= 0)
                throw new ArgumentException("Invalid integration network conditions.");
        }
    }

    internal sealed partial class IntegrationNetManager
    {
        private readonly object _networkGate = new();
        private readonly PriorityQueue<ScheduledPacket, (long Due, long Order)> _scheduledPackets = new();
        private readonly Dictionary<(int Connection, int SequenceChannel), long> _orderedDue = new();
        private IntegrationNetworkConditions? _conditions;
        private TimeSpan _networkTime;
        private long _packetOrder;
        private int _unreliableCount;
        internal int QueuedNetworkBytes { get; private set; }
        internal int DroppedNetworkPackets { get; private set; }
        internal int DelayedNetworkPackets { get; private set; }
        internal int DeliveredDelayedPackets { get; private set; }
        internal TimeSpan? MinimumDeliveredDelay { get; private set; }
        internal TimeSpan MaximumDeliveredDelay { get; private set; }

        private sealed record ScheduledPacket(IntegrationNetChannel Channel, DataMessage Message, TimeSpan Sent);

        internal void ConfigureNetwork(IntegrationNetworkConditions conditions)
        {
            lock (_networkGate) ConfigureNetworkCore(conditions);
        }

        private void ConfigureNetworkCore(IntegrationNetworkConditions conditions)
        {
            conditions.Validate();
            if (_scheduledPackets.Count != 0) throw new InvalidOperationException("Drain scheduled packets before reconfiguring a link.");
            _conditions = conditions;
        }

        /// <summary>Call on both peers before each test tick, with the same elapsed duration.</summary>
        internal void AdvanceNetwork(TimeSpan elapsed)
        {
            lock (_networkGate) AdvanceNetworkCore(elapsed);
        }

        private void AdvanceNetworkCore(TimeSpan elapsed)
        {
            if (elapsed < TimeSpan.Zero) throw new ArgumentOutOfRangeException(nameof(elapsed));
            _networkTime += elapsed;
            while (_scheduledPackets.TryPeek(out _, out var when) && when.Due <= _networkTime.Ticks)
            {
                var packet = _scheduledPackets.Dequeue();
                QueuedNetworkBytes -= packet.Message.Length;
                if (!packet.Channel.IsConnected || !packet.Channel.OtherChannel.TryWrite(packet.Message))
                    ArrayPool<byte>.Shared.Return(packet.Message.PooledNetBuffer);
                else
                {
                    DeliveredDelayedPackets++;
                    var delay = _networkTime - packet.Sent;
                    if (MinimumDeliveredDelay == null || delay < MinimumDeliveredDelay) MinimumDeliveredDelay = delay;
                    if (delay > MaximumDeliveredDelay) MaximumDeliveredDelay = delay;
                }
            }
        }

        private void SendPacket(NetMessage message, IntegrationNetChannel channel)
        {
            lock (_networkGate) SendPacketCore(message, channel);
        }

        private void SendPacketCore(NetMessage message, IntegrationNetChannel channel)
        {
            var data = SerializeNetMessage(message, channel.RemoteUid);
            if (_conditions is not { } conditions)
            {
                if (!channel.OtherChannel.TryWrite(data)) ArrayPool<byte>.Shared.Return(data.PooledNetBuffer);
                return;
            }
            var order = checked(++_packetOrder);
            var unreliable = message.DeliveryMethod is NetDeliveryMethod.Unreliable or NetDeliveryMethod.UnreliableSequenced;
            if (unreliable && conditions.DropEveryNthUnreliable > 0 && ++_unreliableCount % conditions.DropEveryNthUnreliable == 0)
            {
                DroppedNetworkPackets++;
                ArrayPool<byte>.Shared.Return(data.PooledNetBuffer);
                return;
            }
            if (data.Length > conditions.MaximumQueuedBytes - QueuedNetworkBytes)
            {
                ArrayPool<byte>.Shared.Return(data.PooledNetBuffer);
                throw new InvalidOperationException("Integration network queue exceeded its byte budget.");
            }
            // A fixed, non-random jitter pattern makes reordering reproducible.
            var jitter = (order % 4) switch { 0 => -conditions.Jitter.Ticks, 2 => conditions.Jitter.Ticks, _ => 0 };
            var due = checked(_networkTime.Ticks + conditions.OneWayDelay.Ticks + jitter);
            if (message.DeliveryMethod == NetDeliveryMethod.ReliableOrdered)
            {
                var key = (channel.ConnectionUid, message.SequenceChannel);
                if (_orderedDue.TryGetValue(key, out var previous)) due = Math.Max(due, previous);
                _orderedDue[key] = due;
            }
            _scheduledPackets.Enqueue(new ScheduledPacket(channel, data, _networkTime), (due, order));
            QueuedNetworkBytes += data.Length;
            DelayedNetworkPackets++;
        }

        private void ClearScheduledPackets()
        {
            lock (_networkGate) ClearScheduledPacketsCore();
        }

        private void ClearScheduledPacketsCore()
        {
            while (_scheduledPackets.TryDequeue(out var packet, out _)) ArrayPool<byte>.Shared.Return(packet.Message.PooledNetBuffer);
            QueuedNetworkBytes = 0;
            _orderedDue.Clear();
        }

        private void CancelScheduledPackets(int connection)
        {
            lock (_networkGate) CancelScheduledPacketsCore(connection);
        }

        private void CancelScheduledPacketsCore(int connection)
        {
            var retained = new List<(ScheduledPacket Packet, (long Due, long Order) When)>();
            while (_scheduledPackets.TryDequeue(out var packet, out var when))
            {
                if (packet.Channel.ConnectionUid != connection) { retained.Add((packet, when)); continue; }
                QueuedNetworkBytes -= packet.Message.Length;
                ArrayPool<byte>.Shared.Return(packet.Message.PooledNetBuffer);
            }
            foreach (var (packet, when) in retained) _scheduledPackets.Enqueue(packet, when);
            foreach (var key in new List<(int Connection, int SequenceChannel)>(_orderedDue.Keys))
                if (key.Connection == connection) _orderedDue.Remove(key);
        }
    }
}
