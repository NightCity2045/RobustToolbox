using System;
using Robust.Shared.GameObjects;
using Robust.Shared.Maths;
using Robust.Shared.Serialization;
using Robust.Shared.Serialization.Manager.Attributes;

namespace Robust.Shared.Spatial;

/// <summary>Direction along a canonical unit edge in the frame's authoring grid.</summary>
public enum BoundaryAxis : byte
{
    Horizontal,
    Vertical,
}

public enum CellBoundary : byte
{
    South,
    East,
    North,
    West,
}

/// <summary>
/// One address for a shared cell boundary. X/Y identify the lesser endpoint;
/// Horizontal extends toward +X, Vertical toward +Y. Coordinates are grid indices,
/// not metres. The owner is a runtime entity reference: map serialization remaps it
/// to a stable file-local reference and remaps it again when loading a prefab.
/// This runtime type is deliberately not a wire type containing raw EntityUid.
/// </summary>
[DataRecord]
public readonly partial record struct BoundaryKey : ISerializationHooks
{
    public readonly EntityUid OwnerFrame;
    public readonly LayerId LayerId;
    public readonly int X;
    public readonly int Y;
    public readonly BoundaryAxis Axis;

    public BoundaryKey(EntityUid ownerFrame, LayerId layerId, int x, int y, BoundaryAxis axis)
    {
        OwnerFrame = ownerFrame;
        LayerId = layerId;
        X = x;
        Y = y;
        Axis = axis;
        Validate();
    }

    public void Validate()
    {
        if (!OwnerFrame.IsValid())
            throw new ArgumentException("A boundary requires an owner frame.", nameof(OwnerFrame));
        if (Axis is not (BoundaryAxis.Horizontal or BoundaryAxis.Vertical))
            throw new ArgumentOutOfRangeException(nameof(Axis));
        if (Axis == BoundaryAxis.Horizontal && X == int.MaxValue)
            throw new ArgumentOutOfRangeException(nameof(X), "Both boundary endpoints must fit in grid coordinates.");
        if (Axis == BoundaryAxis.Vertical && Y == int.MaxValue)
            throw new ArgumentOutOfRangeException(nameof(Y), "Both boundary endpoints must fit in grid coordinates.");
    }

    void ISerializationHooks.AfterDeserialization() => Validate();

    public (Vector2i Start, Vector2i End) Endpoints()
    {
        Validate();
        return (new Vector2i(X, Y), Axis == BoundaryAxis.Horizontal
            ? new Vector2i(checked(X + 1), Y)
            : new Vector2i(X, checked(Y + 1)));
    }

    public static BoundaryKey FromCell(EntityUid frame, LayerId layer, Vector2i cell, CellBoundary side)
        => side switch
        {
            CellBoundary.South => new BoundaryKey(frame, layer, cell.X, cell.Y, BoundaryAxis.Horizontal),
            CellBoundary.East => new BoundaryKey(frame, layer, checked(cell.X + 1), cell.Y, BoundaryAxis.Vertical),
            CellBoundary.North => new BoundaryKey(frame, layer, cell.X, checked(cell.Y + 1), BoundaryAxis.Horizontal),
            CellBoundary.West => new BoundaryKey(frame, layer, cell.X, cell.Y, BoundaryAxis.Vertical),
            _ => throw new ArgumentOutOfRangeException(nameof(side)),
        };

    /// <summary>Normalize either direction of an axis-aligned unit edge.</summary>
    public static BoundaryKey FromEndpoints(EntityUid frame, LayerId layer, Vector2i first, Vector2i second)
    {
        var dx = (long) second.X - first.X;
        var dy = (long) second.Y - first.Y;
        if (dy == 0 && Math.Abs(dx) == 1)
            return new BoundaryKey(frame, layer, Math.Min(first.X, second.X), first.Y, BoundaryAxis.Horizontal);
        if (dx == 0 && Math.Abs(dy) == 1)
            return new BoundaryKey(frame, layer, first.X, Math.Min(first.Y, second.Y), BoundaryAxis.Vertical);
        throw new ArgumentException("A boundary must be an axis-aligned unit edge.");
    }

    /// <summary>
    /// Authoring-grid prefab transform: mirror X first, then rotate counterclockwise
    /// in quarter turns around the origin, then translate. Normalizes the transformed
    /// endpoints so opposite representations cannot create duplicate walls.
    /// Arbitrary world orientations belong to the owner transform and do not change keys.
    /// </summary>
    public BoundaryKey Transform(EntityUid targetFrame, LayerId targetLayer, int quarterTurns,
        bool mirrorX, Vector2i offset)
    {
        if (quarterTurns is < 0 or > 3)
            throw new ArgumentOutOfRangeException(nameof(quarterTurns));
        var (first, second) = Endpoints();
        return FromEndpoints(targetFrame, targetLayer, TransformPoint(first), TransformPoint(second));

        Vector2i TransformPoint(Vector2i point)
        {
            // Use wider intermediates: reflection followed by translation can fit
            // even when the reflected intermediate alone would exceed Int32.
            long x = mirrorX ? -(long) point.X : point.X;
            long y = point.Y;
            (x, y) = quarterTurns switch
            {
                0 => (x, y),
                1 => (-y, x),
                2 => (-x, -y),
                _ => (y, -x),
            };
            return new Vector2i(checked((int) (x + offset.X)), checked((int) (y + offset.Y)));
        }
    }
}
