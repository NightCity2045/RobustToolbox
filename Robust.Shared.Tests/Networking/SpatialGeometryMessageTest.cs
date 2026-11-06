using System.IO;
using System.Numerics;
using Lidgren.Network;
using NUnit.Framework;
using Robust.Shared.GameObjects;
using Robust.Shared.Maths;
using Robust.Shared.Network;
using Robust.Shared.Spatial;
using Robust.Shared.Spatial.Networking;
using Robust.Shared.Timing;

namespace Robust.Shared.Tests.Networking;

[TestFixture]
public sealed class SpatialGeometryMessageTest
{
    private static byte[] Write(NetMessage message)
    {
        var buffer = new NetOutgoingMessage();
        message.WriteToBuffer(buffer, null!);
        return buffer.Data.AsSpan(0, buffer.LengthBytes).ToArray();
    }

    private static void Read(NetMessage message, byte[] bytes)
        => message.ReadFromBuffer(new NetIncomingMessage { Data = bytes, LengthBytes = bytes.Length }, null!);

    [Test]
    public void AllMessagesRoundTripAndRejectEveryTruncationOrTrailingByte()
    {
        var manifest = new SpatialGeometryManifest(2, 1, new GameTick(10), new[]
        {
            new SpatialGeometryFrameManifest(new NetEntity(1), new NetEntity(2),
                new SpatialGeometryStamp(Guid.NewGuid(), 3), new string('A', 64), 100)
        }, new[] { new SpatialGeometryCoverage(new NetEntity(1), new Box3(Vector3.Zero, Vector3.One)) });
        NetMessage[] messages =
        [
            new MsgSpatialGeometryManifest { Manifest = manifest },
            new MsgSpatialGeometryFragment { Sequence = 2, Frame = new NetEntity(2), Offset = 0, Data = new byte[100] },
            new MsgSpatialGeometryAck { Sequence = 2 },
            new MsgSpatialGeometryProgress { Sequence = 2, Frame = new NetEntity(2), Offset = 0 },
            new MsgSpatialGeometryRequestBaseline { LastSequence = 1 }
        ];
        foreach (var message in messages)
        {
            var bytes = Write(message);
            NetMessage Fresh() => (NetMessage) Activator.CreateInstance(message.GetType())!;
            var read = Fresh();
            Read(read, bytes);
            Assert.That(Write(read), Is.EqualTo(bytes));
            for (var length = 0; length < bytes.Length; length++)
                Assert.Catch(() => Read(Fresh(), bytes[..length]), $"{message.GetType().Name} truncated at {length}");
            Assert.Throws<InvalidDataException>(() => Read(Fresh(), bytes.Append((byte) 0).ToArray()));
        }
    }

    [Test]
    public void InvalidFragmentAndForgedProgressBoundsAreRejected()
    {
        foreach (var offset in new[] { -1, 1, SpatialGeometryWireCodec.MaximumBytes })
        {
            var bytes = Write(new MsgSpatialGeometryProgress { Sequence = 1, Frame = new NetEntity(1), Offset = offset });
            Assert.Throws<InvalidDataException>(() => Read(new MsgSpatialGeometryProgress(), bytes));
        }
        var invalid = Write(new MsgSpatialGeometryFragment { Sequence = 0, Frame = new NetEntity(1), Data = new byte[1] });
        Assert.Throws<InvalidDataException>(() => Read(new MsgSpatialGeometryFragment(), invalid));
    }
}
