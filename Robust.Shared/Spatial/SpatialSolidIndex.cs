using System;
using System.Collections.Generic;
using System.Numerics;
using Robust.Shared.Maths;

namespace Robust.Shared.Spatial;

/// <summary>Temporary balanced volume index for snapshot geometry jobs, not authoritative state.</summary>
internal sealed class SpatialSolidIndex
{
    private readonly Node[] _nodes;
    private readonly IReadOnlyList<SpatialSolidPart> _solids;
    private readonly int[] _indices;
    private int _next;

    private struct Node
    {
        public Vector3 Min;
        public Vector3 Max;
        public int Left;
        public int Right;
        public int Solid;
    }

    public SpatialSolidIndex(IReadOnlyList<SpatialSolidPart> solids)
    {
        _solids = solids;
        _indices = new int[solids.Count];
        _nodes = new Node[solids.Count == 0 ? 0 : checked(solids.Count * 2 - 1)];
        for (var i = 0; i < _indices.Length; i++)
            _indices[i] = i;
        if (solids.Count > 0)
            Build(0, solids.Count);
    }

    private int Build(int start, int count)
    {
        var nodeIndex = _next++;
        var min = _solids[_indices[start]].Bounds.Min;
        var max = _solids[_indices[start]].Bounds.Max;
        for (var i = start + 1; i < start + count; i++)
        {
            var box = _solids[_indices[i]].Bounds;
            min = Vector3.Min(min, box.Min);
            max = Vector3.Max(max, box.Max);
        }
        _nodes[nodeIndex] = new Node { Min = min, Max = max, Solid = -1 };
        if (count == 1)
        {
            _nodes[nodeIndex].Solid = _indices[start];
            return nodeIndex;
        }

        var x = (double) max.X - min.X;
        var y = (double) max.Y - min.Y;
        var z = (double) max.Z - min.Z;
        var axis = x >= y && x >= z ? 0 : y >= z ? 1 : 2;
        Array.Sort(_indices, start, count, Comparer<int>.Create((a, b) =>
        {
            var centerA = _solids[a].Bounds.Center;
            var centerB = _solids[b].Bounds.Center;
            var result = axis switch
            {
                0 => centerA.X.CompareTo(centerB.X),
                1 => centerA.Y.CompareTo(centerB.Y),
                _ => centerA.Z.CompareTo(centerB.Z)
            };
            return result != 0 ? result : a.CompareTo(b);
        }));
        var half = count / 2;
        _nodes[nodeIndex].Left = Build(start, half);
        _nodes[nodeIndex].Right = Build(start + half, count - half);
        return nodeIndex;
    }

    public void FindIntersections(Box3 bounds, List<int> result, Stack<int> pending)
    {
        result.Clear();
        pending.Clear();
        if (_nodes.Length == 0)
            return;
        pending.Push(0);
        while (pending.TryPop(out var index))
        {
            ref var node = ref _nodes[index];
            if (bounds.Min.X > node.Max.X || bounds.Max.X < node.Min.X
                || bounds.Min.Y > node.Max.Y || bounds.Max.Y < node.Min.Y
                || bounds.Min.Z > node.Max.Z || bounds.Max.Z < node.Min.Z)
                continue;
            if (node.Solid >= 0)
                result.Add(node.Solid);
            else
            {
                pending.Push(node.Left);
                pending.Push(node.Right);
            }
        }
    }
}
