using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using Robust.Shared.Maths;
using Robust.Shared.Serialization;
using Robust.Shared.Serialization.Manager.Attributes;

namespace Robust.Shared.Spatial;

/// <summary>A local chunk address, scoped by the geometry's owner frame.</summary>
[DataRecord]
public readonly partial record struct SpatialChunkKey(LayerId Layer, Vector2i Indices)
{
    // Part of spatial geometry schema 1. There is no vertical chunk dimension:
    // authoring layers have explicit continuous elevations in the frame.
    public const int Size = 16;

    public static SpatialChunkKey FromCell(LayerId layer, Vector2i cell)
        => new(layer, new Vector2i(cell.X >> 4, cell.Y >> 4));

    public static Vector2i LocalCell(Vector2i cell)
        => new(cell.X & (Size - 1), cell.Y & (Size - 1));

    public Vector2i FrameCell(Vector2i local)
    {
        ValidateLocal(local);
        return new Vector2i(checked(Indices.X * Size + local.X), checked(Indices.Y * Size + local.Y));
    }

    internal static void ValidateLocal(Vector2i local)
    {
        if (local.X < 0 || local.X >= Size || local.Y < 0 || local.Y >= Size)
            throw new ArgumentOutOfRangeException(nameof(local), "A tile must lie inside its chunk.");
    }
}

/// <summary>
/// Immutable sparse chunk of decorative tiles. Empty chunks are absent from the
/// canonical store; distant cells and unused layers allocate no intervening data.
/// </summary>
[DataDefinition]
public sealed partial class SpatialTileChunk : ISerializationHooks
{
    [DataField("key", required: true)] public SpatialChunkKey Key { get; private set; }
    [DataField("revision")] public ulong StateRevision { get; private set; }
    [DataField("tiles", required: true)] private Dictionary<Vector2i, SpatialFloorTile> _tiles = new();

    public IReadOnlyDictionary<Vector2i, SpatialFloorTile> Tiles => new ReadOnlyDictionary<Vector2i, SpatialFloorTile>(_tiles);

    public bool TryGetTile(Vector2i local, out SpatialFloorTile tile)
    {
        SpatialChunkKey.ValidateLocal(local);
        return _tiles.TryGetValue(local, out tile);
    }

    private SpatialTileChunk() { }

    public SpatialTileChunk(SpatialChunkKey key, IEnumerable<KeyValuePair<Vector2i, SpatialFloorTile>> tiles,
        ulong stateRevision = 0)
    {
        ArgumentNullException.ThrowIfNull(tiles);
        Key = key;
        StateRevision = stateRevision;
        foreach (var (cell, tile) in tiles)
            _tiles.Add(cell, tile);
        Validate();
    }

    public void Validate()
    {
        if (_tiles == null || _tiles.Count == 0 || _tiles.Count > SpatialChunkKey.Size * SpatialChunkKey.Size)
            throw new ArgumentException("A stored tile chunk must contain 1 to 256 occupied cells.");
        foreach (var (local, tile) in _tiles)
        {
            Key.FrameCell(local);
            tile.Validate();
        }
    }

    void ISerializationHooks.AfterDeserialization() => Validate();
}
