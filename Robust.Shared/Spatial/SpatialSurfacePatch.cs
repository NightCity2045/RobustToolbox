using System;
using System.Numerics;
using Robust.Shared.Maths;

namespace Robust.Shared.Spatial;

public enum SpatialFaceDirection : byte
{
    NegativeX,
    PositiveX,
    NegativeY,
    PositiveY,
    NegativeZ,
    PositiveZ
}

/// <summary>
/// A rectangular exterior face in frame-local metres. The source retains semantic
/// identity after clipping or batching. Vertices 0,1,2 and 0,2,3 wind outward.
/// This CPU geometry owns no graphics resources or gameplay material properties.
/// </summary>
public readonly record struct SpatialSurfacePatch
{
    public SpatialSolidPart Source { get; }
    public SpatialFaceDirection Direction { get; }
    public float Plane { get; }
    public Box2 Region { get; }
    public double Area => (double) Region.Width * Region.Height;
    public Vector3 Normal => Direction switch
    {
        SpatialFaceDirection.NegativeX => -Vector3.UnitX,
        SpatialFaceDirection.PositiveX => Vector3.UnitX,
        SpatialFaceDirection.NegativeY => -Vector3.UnitY,
        SpatialFaceDirection.PositiveY => Vector3.UnitY,
        SpatialFaceDirection.NegativeZ => -Vector3.UnitZ,
        SpatialFaceDirection.PositiveZ => Vector3.UnitZ,
        _ => throw new ArgumentOutOfRangeException()
    };

    internal SpatialSurfacePatch(SpatialSolidPart source, SpatialFaceDirection direction, float plane, Box2 region)
    {
        Source = source;
        Direction = direction;
        Plane = plane;
        Region = region;
    }

    public Vector3 GetVertex(int index)
    {
        if (index < 0 || index > 3)
            throw new ArgumentOutOfRangeException(nameof(index));
        // Projection on (Y,Z), (X,Z), (X,Y) has normals +X, -Y, +Z.
        if (Direction is SpatialFaceDirection.NegativeX or SpatialFaceDirection.PositiveY or SpatialFaceDirection.NegativeZ)
            index = (4 - index) % 4;
        var u = index is 0 or 3 ? Region.Left : Region.Right;
        var v = index is 0 or 1 ? Region.Bottom : Region.Top;
        return ((int) Direction / 2) switch
        {
            0 => new Vector3(Plane, u, v),
            1 => new Vector3(u, Plane, v),
            2 => new Vector3(u, v, Plane),
            _ => throw new ArgumentOutOfRangeException()
        };
    }
}
