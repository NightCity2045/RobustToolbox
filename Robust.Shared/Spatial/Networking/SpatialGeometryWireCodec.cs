using System;
using System.Buffers.Binary;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using Robust.Shared.GameObjects;
using Robust.Shared.Maths;

namespace Robust.Shared.Spatial.Networking;

/// <summary>
/// Bounded semantic geometry payload. Entity references use a separate NetEntity
/// table; decoding resolves that table before constructing any geometry. The
/// resulting detached snapshot has no live capture identity and grants no edits.
/// Transport generation, world, tick and baseline ordering belong to the envelope.
/// </summary>
internal static class SpatialGeometryWireCodec
{
    private const uint Magic = 0x53474731;
    internal const uint Version = 1;
    internal const int MaximumBytes = 8 * 1024 * 1024;
    private const int MaximumRecords = 16384;
    private const int MaximumStringBytes = 4096;
    private const long MaximumPartitionWork = 4_000_000;
    private static readonly UTF8Encoding Utf8 = new(false, true);

    /// <summary>Select authorized surfaces and only their decorative coverings.</summary>
    internal static SpatialGeometrySnapshot Select(SpatialGeometrySnapshot source, IReadOnlySet<SurfaceId> surfaces)
    {
        ArgumentNullException.ThrowIfNull(source);
        ArgumentNullException.ThrowIfNull(surfaces);
        var boundaries = source.Boundaries.Values.Where(b => surfaces.Contains(b.Surface)).ToArray();
        var slabs = source.Slabs.Values.Where(s => surfaces.Contains(s.Surface.Surface)).ToArray();
        var layers = boundaries.Select(b => b.Key.LayerId).Concat(slabs.Select(s => s.Layer)).ToHashSet();
        var chunks = new List<SpatialTileChunk>();
        foreach (var chunk in source.TileChunks.Values)
        {
            var tiles = chunk.Tiles.Where(t => surfaces.Contains(t.Value.StructuralSurface)).ToArray();
            if (tiles.Length > 0) chunks.Add(new SpatialTileChunk(chunk.Key, tiles, chunk.StateRevision));
        }
        return new SpatialGeometrySnapshot(source.OwnerFrame, source.Revision,
            source.Layers.Where(p => layers.Contains(p.Key)).ToDictionary(), boundaries, slabs, chunks);
    }

    internal static byte[] Encode(SpatialGeometrySnapshot snapshot, Func<EntityUid, NetEntity> getNetEntity)
    {
        ArgumentNullException.ThrowIfNull(snapshot);
        ArgumentNullException.ThrowIfNull(getNetEntity);
        var references = new Dictionary<EntityUid, int> { [snapshot.OwnerFrame] = 0 };
        foreach (var source in snapshot.Boundaries.Values.OrderBy(b => b.Surface.Value).Select(b => b.Entity)
                     .Concat(snapshot.Slabs.Values.OrderBy(s => s.Surface.Surface.Value).Select(s => s.Entity)))
            if (source is { } uid && !references.ContainsKey(uid)) references.Add(uid, references.Count);
        using var stream = new MemoryStream();
        using var writer = new BinaryWriter(stream, Utf8, leaveOpen: true);
        writer.Write(Magic);
        writer.Write(Version);
        Count(references.Count);
        var netIds = new HashSet<NetEntity>();
        foreach (var uid in references.Keys)
        {
            var net = getNetEntity(uid);
            ValidateNet(net);
            if (!netIds.Add(net)) throw new InvalidDataException("Geometry entity mapping must be injective.");
            writer.Write(net.Id);
        }
        writer.Write(snapshot.Revision);
        Count(snapshot.Layers.Count);
        foreach (var (id, layer) in snapshot.Layers.OrderBy(p => p.Key.Value))
        {
            writer.Write(id.Value);
            writer.Write(layer.LocalElevation);
            writer.Write(layer.AuthoringGrid);
            Text(layer.DisplayName);
        }
        long work = 0;
        Count(snapshot.Boundaries.Count);
        foreach (var boundary in snapshot.Boundaries.Values.OrderBy(b => b.Surface.Value))
        {
            writer.Write(boundary.Surface.Value);
            writer.Write(boundary.Key.LayerId.Value);
            writer.Write(boundary.Key.X);
            writer.Write(boundary.Key.Y);
            writer.Write((byte) boundary.Key.Axis);
            writer.Write(boundary.Thickness);
            writer.Write(boundary.BaseElevation);
            writer.Write(boundary.Height);
            Reference(boundary.Entity);
            writer.Write(boundary.StateRevision);
            Text(boundary.MaterialProfile);
            PartitionBudget(boundary.Openings.Count, ref work);
            Count(boundary.Openings.Count);
            foreach (var opening in boundary.Openings)
            {
                writer.Write(opening.Start); writer.Write(opening.End);
                writer.Write(opening.Bottom); writer.Write(opening.Top);
            }
            Size();
        }
        Count(snapshot.Slabs.Count);
        foreach (var slab in snapshot.Slabs.Values.OrderBy(s => s.Surface.Surface.Value))
        {
            writer.Write(slab.Surface.Surface.Value);
            writer.Write(slab.Layer.Value);
            Rectangle(slab.Footprint);
            writer.Write(slab.TopElevation);
            writer.Write(slab.Thickness);
            Reference(slab.Entity);
            writer.Write(slab.StateRevision);
            Text(slab.MaterialProfile);
            PartitionBudget(slab.Openings.Count, ref work);
            Count(slab.Openings.Count);
            foreach (var opening in slab.Openings) Rectangle(opening);
            Size();
        }
        Count(snapshot.TileChunks.Count);
        foreach (var chunk in snapshot.TileChunks.Values.OrderBy(c => c.Key.Layer.Value)
                     .ThenBy(c => c.Key.Indices.X).ThenBy(c => c.Key.Indices.Y))
        {
            writer.Write(chunk.Key.Layer.Value);
            writer.Write(chunk.Key.Indices.X);
            writer.Write(chunk.Key.Indices.Y);
            writer.Write(chunk.StateRevision);
            Count(chunk.Tiles.Count);
            foreach (var (cell, tile) in chunk.Tiles.OrderBy(t => t.Key.X).ThenBy(t => t.Key.Y))
            {
                writer.Write((byte) cell.X); writer.Write((byte) cell.Y);
                writer.Write(tile.StructuralSurface.Value);
                Text(tile.MaterialProfile);
            }
            Size();
        }
        Size();
        return stream.ToArray();

        void Count(int count)
        {
            if (count < 0 || count > MaximumRecords) throw new InvalidDataException("Geometry record budget exceeded.");
            writer.Write(count);
        }
        void Text(string text)
        {
            var size = Utf8.GetByteCount(text);
            if (size > MaximumStringBytes) throw new InvalidDataException("Geometry string budget exceeded.");
            writer.Write(size);
            writer.Write(Utf8.GetBytes(text));
            Size();
        }
        void Reference(EntityUid? uid) => writer.Write(uid is { } value ? references[value] : -1);
        void Rectangle(Box2 box)
        {
            writer.Write(box.Left); writer.Write(box.Bottom); writer.Write(box.Right); writer.Write(box.Top);
        }
        void Size()
        {
            if (stream.Length > MaximumBytes) throw new InvalidDataException("Geometry byte budget exceeded; split the authorized selection.");
        }
    }

    /// <summary>
    /// The resolver must return initialized entities in the envelope's world.
    /// An unavailable reference should throw so transport can defer the complete
    /// payload, rather than creating a parent or binding geometry at the origin.
    /// </summary>
    internal static SpatialGeometrySnapshot Decode(byte[] bytes, Func<NetEntity, EntityUid> resolve)
    {
        ArgumentNullException.ThrowIfNull(bytes);
        ArgumentNullException.ThrowIfNull(resolve);
        var netReferences = ReadReferences(bytes);
        using var stream = new MemoryStream(bytes, writable: false);
        using var reader = new BinaryReader(stream, Utf8);
        stream.Position = 12 + netReferences.Length * 4;
        var referenceCount = netReferences.Length;
        var references = new EntityUid[referenceCount];
        var localIds = new HashSet<EntityUid>();
        for (var i = 0; i < referenceCount; i++)
        {
            var uid = resolve(netReferences[i]);
            if (!uid.IsValid() || !localIds.Add(uid)) throw new InvalidDataException("Invalid or ambiguous geometry entity mapping.");
            references[i] = uid;
        }
        var frame = references[0];
        var revision = reader.ReadUInt64();
        var layers = new Dictionary<LayerId, LayerDefinition>();
        var count = Count(16);
        for (var i = 0; i < count; i++)
        {
            var id = new LayerId(reader.ReadUInt32());
            var elevation = reader.ReadSingle();
            var grid = reader.ReadSingle();
            layers.Add(id, new LayerDefinition(elevation, Text(), grid));
        }
        long work = 0;
        var boundaries = new List<BoundarySegment>();
        count = Count(45);
        for (var i = 0; i < count; i++)
        {
            var surface = new SurfaceId(reader.ReadUInt32());
            var layer = new LayerId(reader.ReadUInt32());
            var key = new BoundaryKey(frame, layer, reader.ReadInt32(), reader.ReadInt32(), (BoundaryAxis) reader.ReadByte());
            var thickness = reader.ReadSingle();
            var bottom = reader.ReadSingle();
            var height = reader.ReadSingle();
            var source = Reference();
            var stateRevision = reader.ReadUInt64();
            var material = Text();
            var holes = Count(16);
            PartitionBudget(holes, ref work);
            var openings = new BoundaryOpening[holes];
            for (var h = 0; h < holes; h++) openings[h] = new BoundaryOpening(
                reader.ReadSingle(), reader.ReadSingle(), reader.ReadSingle(), reader.ReadSingle());
            boundaries.Add(new BoundarySegment(key, surface, thickness, bottom, height, openings, material, source, stateRevision));
        }
        var slabs = new List<SlabSurface>();
        count = Count(52);
        for (var i = 0; i < count; i++)
        {
            var surface = new SpatialSurfaceReference(frame, new SurfaceId(reader.ReadUInt32()));
            var layer = new LayerId(reader.ReadUInt32());
            var footprint = Rectangle();
            var top = reader.ReadSingle();
            var thickness = reader.ReadSingle();
            var source = Reference();
            var stateRevision = reader.ReadUInt64();
            var material = Text();
            var holes = Count(16);
            PartitionBudget(holes, ref work);
            var openings = new Box2[holes];
            for (var h = 0; h < holes; h++) openings[h] = Rectangle();
            slabs.Add(new SlabSurface(surface, layer, footprint, top, thickness, openings, material, source, stateRevision));
        }
        var chunks = new List<SpatialTileChunk>();
        count = Count(24);
        for (var i = 0; i < count; i++)
        {
            var key = new SpatialChunkKey(new LayerId(reader.ReadUInt32()), new Vector2i(reader.ReadInt32(), reader.ReadInt32()));
            var chunkRevision = reader.ReadUInt64();
            var tiles = Count(10, SpatialChunkKey.Size * SpatialChunkKey.Size);
            var entries = new Dictionary<Vector2i, SpatialFloorTile>();
            for (var t = 0; t < tiles; t++)
            {
                var cell = new Vector2i(reader.ReadByte(), reader.ReadByte());
                var surface = new SurfaceId(reader.ReadUInt32());
                entries.Add(cell, new SpatialFloorTile(Text(), surface));
            }
            chunks.Add(new SpatialTileChunk(key, entries, chunkRevision));
        }
        if (stream.Position != stream.Length) throw new InvalidDataException("Trailing spatial geometry data.");
        return new SpatialGeometrySnapshot(frame, revision, layers, boundaries, slabs, chunks);

        int Count(int minimumBytes, int limit = MaximumRecords)
        {
            var value = reader.ReadInt32();
            if (value < 0 || value > limit || value > (stream.Length - stream.Position) / minimumBytes)
                throw new InvalidDataException("Invalid spatial geometry record count.");
            return value;
        }
        string Text()
        {
            var length = Count(1, MaximumStringBytes);
            return Utf8.GetString(reader.ReadBytes(length));
        }
        EntityUid? Reference()
        {
            var index = reader.ReadInt32();
            if (index == -1) return null;
            if ((uint) index >= references.Length) throw new InvalidDataException("Unknown geometry entity reference.");
            return references[index];
        }
        Box2 Rectangle() => new(reader.ReadSingle(), reader.ReadSingle(), reader.ReadSingle(), reader.ReadSingle());
    }

    internal static NetEntity[] ReadReferences(byte[] bytes)
    {
        if (bytes.Length < 12 || bytes.Length > MaximumBytes
            || BinaryPrimitives.ReadUInt32LittleEndian(bytes) != Magic
            || BinaryPrimitives.ReadUInt32LittleEndian(bytes.AsSpan(4)) != Version)
            throw new InvalidDataException("Invalid spatial geometry payload header.");
        var count = BinaryPrimitives.ReadInt32LittleEndian(bytes.AsSpan(8));
        if (count <= 0 || count > MaximumRecords || count > (bytes.Length - 12) / 4)
            throw new InvalidDataException("Invalid geometry reference count.");
        var references = new NetEntity[count];
        var seen = new HashSet<NetEntity>();
        for (var i = 0; i < count; i++)
        {
            var net = new NetEntity(BinaryPrimitives.ReadInt32LittleEndian(bytes.AsSpan(12 + i * 4)));
            ValidateNet(net);
            if (!seen.Add(net)) throw new InvalidDataException("Duplicate geometry network reference.");
            references[i] = net;
        }
        return references;
    }

    private static void ValidateNet(NetEntity net)
    {
        if (net.Id <= 0 || net.Id >= NetEntity.ClientEntity)
            throw new InvalidDataException("Geometry references must be server network entity IDs.");
    }

    private static void PartitionBudget(int holes, ref long work)
    {
        work += (2L * holes + 1) * (holes + 1);
        if (holes < 0 || work > MaximumPartitionWork)
            throw new InvalidDataException("Geometry partition work budget exceeded; split the authorized selection.");
    }
}
