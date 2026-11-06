using System;
using Robust.Shared.Serialization;
using Robust.Shared.Serialization.Manager.Attributes;

namespace Robust.Shared.Spatial;

/// <summary>A real rectangular void, measured along the boundary and above its base in metres.</summary>
[DataRecord]
public readonly partial record struct BoundaryOpening : ISerializationHooks
{
    public readonly float Start;
    public readonly float End;
    public readonly float Bottom;
    public readonly float Top;

    public BoundaryOpening(float start, float end, float bottom, float top)
    {
        Start = start;
        End = end;
        Bottom = bottom;
        Top = top;
        Validate();
    }

    public void Validate()
    {
        if (!float.IsFinite(Start) || !float.IsFinite(End) || !float.IsFinite(Bottom) || !float.IsFinite(Top)
            || Start < 0 || Bottom < 0 || End <= Start || Top <= Bottom)
            throw new ArgumentException("An opening requires finite, positive along-edge and height intervals.");
    }

    void ISerializationHooks.AfterDeserialization() => Validate();
}
