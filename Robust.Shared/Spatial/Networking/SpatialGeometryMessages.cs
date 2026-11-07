using System;
using System.Collections.Generic;
using System.IO;
using System.Numerics;
using Lidgren.Network;
using Robust.Shared.GameObjects;
using Robust.Shared.Maths;
using Robust.Shared.Network;
using Robust.Shared.Serialization;
using Robust.Shared.Timing;

namespace Robust.Shared.Spatial.Networking;

internal abstract class SpatialGeometryMessage : NetMessage
{
    public override MsgGroups MsgGroup => MsgGroups.Entity;
    public override NetDeliveryMethod DeliveryMethod => NetDeliveryMethod.ReliableOrdered;
    public override int SequenceChannel => SequenceChannels.SpatialGeometry;

    protected static int Count(NetIncomingMessage buffer, int maximum, int minimumBytes)
    {
        var count = buffer.ReadInt32();
        if (count < 0 || count > maximum || (long) count * minimumBytes * 8 > buffer.LengthBits - buffer.Position)
            throw new InvalidDataException("Invalid geometry message count.");
        return count;
    }

    protected static void End(NetIncomingMessage buffer)
    {
        if (buffer.Position != buffer.LengthBits) throw new InvalidDataException("Trailing geometry message data.");
    }
}

internal sealed class MsgSpatialGeometryManifest : SpatialGeometryMessage
{
    public SpatialGeometryManifest Manifest = default!;

    public override void ReadFromBuffer(NetIncomingMessage buffer, IRobustSerializer serializer)
    {
        var sequence = buffer.ReadUInt64();
        var baseline = buffer.ReadUInt64();
        var tick = new GameTick(buffer.ReadUInt32());
        var frames = new List<SpatialGeometryFrameManifest>();
        var count = Count(buffer, SpatialGeometryManifest.MaximumParts, 76);
        for (var i = 0; i < count; i++)
        {
            var world = new NetEntity(buffer.ReadInt32());
            var frame = new NetEntity(buffer.ReadInt32());
            var generation = new Guid(buffer.ReadBytes(16));
            var revision = buffer.ReadUInt64();
            var hash = Convert.ToHexString(buffer.ReadBytes(32));
            var length = buffer.ReadInt32();
            var part = buffer.ReadInt32();
            var parts = buffer.ReadInt32();
            frames.Add(new SpatialGeometryFrameManifest(world, frame, new SpatialGeometryStamp(generation, revision), hash, length, part, parts));
        }
        var coverage = new List<SpatialGeometryCoverage>();
        count = Count(buffer, SpatialGeometryManifest.MaximumViews, 28);
        for (var i = 0; i < count; i++)
        {
            var world = new NetEntity(buffer.ReadInt32());
            var min = new Vector3(buffer.ReadFloat(), buffer.ReadFloat(), buffer.ReadFloat());
            var max = new Vector3(buffer.ReadFloat(), buffer.ReadFloat(), buffer.ReadFloat());
            coverage.Add(new SpatialGeometryCoverage(world, new Box3(min, max)));
        }
        End(buffer);
        Manifest = new SpatialGeometryManifest(sequence, baseline, tick, frames, coverage);
    }

    public override void WriteToBuffer(NetOutgoingMessage buffer, IRobustSerializer serializer)
    {
        buffer.Write(Manifest.Sequence); buffer.Write(Manifest.BaseSequence); buffer.Write(Manifest.Tick.Value);
        buffer.Write(Manifest.Frames.Count);
        foreach (var frame in Manifest.Frames)
        {
            buffer.Write(frame.World.Id); buffer.Write(frame.Frame.Id);
            buffer.Write(frame.Stamp.Generation.ToByteArray()); buffer.Write(frame.Stamp.Revision);
            buffer.Write(Convert.FromHexString(frame.Hash)); buffer.Write(frame.Length);
            buffer.Write(frame.Part); buffer.Write(frame.Parts);
        }
        buffer.Write(Manifest.Coverage.Count);
        foreach (var view in Manifest.Coverage)
        {
            buffer.Write(view.World.Id);
            buffer.Write(view.Bounds.Min.X); buffer.Write(view.Bounds.Min.Y); buffer.Write(view.Bounds.Min.Z);
            buffer.Write(view.Bounds.Max.X); buffer.Write(view.Bounds.Max.Y); buffer.Write(view.Bounds.Max.Z);
        }
    }

    public override int EstimateBufferSize() => 28 + 76 * Manifest.Frames.Count + 28 * Manifest.Coverage.Count;
}

internal sealed class MsgSpatialGeometryFragment : SpatialGeometryMessage
{
    public ulong Sequence;
    public NetEntity Frame;
    public int Part;
    public int Offset;
    public byte[] Data = Array.Empty<byte>();

    public override void ReadFromBuffer(NetIncomingMessage buffer, IRobustSerializer serializer)
    {
        Sequence = buffer.ReadUInt64(); Frame = new NetEntity(buffer.ReadInt32()); Part = buffer.ReadInt32(); Offset = buffer.ReadInt32();
        var count = Count(buffer, SpatialGeometryReplica.FragmentSize, 1);
        if (Sequence == 0 || !SpatialGeometryManifest.ValidNet(Frame) || Part < 0 || Part >= SpatialGeometryManifest.MaximumParts || Offset < 0
            || Offset >= SpatialGeometryWireCodec.MaximumBytes || Offset % SpatialGeometryReplica.FragmentSize != 0 || count == 0)
            throw new InvalidDataException("Invalid geometry fragment header.");
        Data = buffer.ReadBytes(count);
        End(buffer);
    }

    public override void WriteToBuffer(NetOutgoingMessage buffer, IRobustSerializer serializer)
    {
        if (Data.Length == 0 || Data.Length > SpatialGeometryReplica.FragmentSize) throw new InvalidDataException("Invalid fragment length.");
        buffer.Write(Sequence); buffer.Write(Frame.Id); buffer.Write(Part); buffer.Write(Offset); buffer.Write(Data.Length); buffer.Write(Data);
    }
    public override int EstimateBufferSize() => 24 + Data.Length;
}

internal sealed class MsgSpatialGeometryAck : SpatialGeometryMessage
{
    public ulong Sequence;
    public override void ReadFromBuffer(NetIncomingMessage buffer, IRobustSerializer serializer)
    {
        Sequence = buffer.ReadUInt64();
        if (Sequence == 0) throw new InvalidDataException("Invalid geometry acknowledgement.");
        End(buffer);
    }
    public override void WriteToBuffer(NetOutgoingMessage buffer, IRobustSerializer serializer) => buffer.Write(Sequence);
}

internal sealed class MsgSpatialGeometryProgress : SpatialGeometryMessage
{
    public ulong Sequence;
    public NetEntity Frame;
    public int Part;
    public int Offset;
    public override void ReadFromBuffer(NetIncomingMessage buffer, IRobustSerializer serializer)
    {
        Sequence = buffer.ReadUInt64(); Frame = new NetEntity(buffer.ReadInt32()); Part = buffer.ReadInt32(); Offset = buffer.ReadInt32();
        if (Sequence == 0 || !SpatialGeometryManifest.ValidNet(Frame) || Part < 0 || Part >= SpatialGeometryManifest.MaximumParts || Offset < 0
            || Offset >= SpatialGeometryWireCodec.MaximumBytes || Offset % SpatialGeometryReplica.FragmentSize != 0)
            throw new InvalidDataException("Invalid geometry progress acknowledgement.");
        End(buffer);
    }
    public override void WriteToBuffer(NetOutgoingMessage buffer, IRobustSerializer serializer)
    {
        buffer.Write(Sequence); buffer.Write(Frame.Id); buffer.Write(Part); buffer.Write(Offset);
    }
}

internal sealed class MsgSpatialGeometryRequestBaseline : SpatialGeometryMessage
{
    public ulong LastSequence;
    public override void ReadFromBuffer(NetIncomingMessage buffer, IRobustSerializer serializer)
    {
        LastSequence = buffer.ReadUInt64(); End(buffer);
    }
    public override void WriteToBuffer(NetOutgoingMessage buffer, IRobustSerializer serializer) => buffer.Write(LastSequence);
}
