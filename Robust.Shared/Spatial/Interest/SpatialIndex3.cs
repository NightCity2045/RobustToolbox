using System;
using System.Buffers;
using System.Collections.Generic;
using System.Numerics;
using BepuPhysics.Trees;
using BepuUtilities;
using BepuUtilities.Memory;

namespace Robust.Shared.Spatial.Interest;

/// <summary>
/// Derived volume index. Writers run before the parallel, read-only interest phase.
/// Contains no poses, gameplay state, network identifiers or visibility decisions.
/// </summary>
internal sealed class SpatialIndex3<TKey> : IDisposable where TKey : notnull
{
    private readonly BufferPool _pool = new();
    private Tree _tree;
    private readonly Dictionary<TKey, int> _leaves = new();
    private readonly List<TKey> _keys = new();
    private bool _disposed;

    public int Count => _keys.Count;

    public SpatialIndex3() => _tree = new Tree(_pool, 16);

    /// <summary>Bounds may be degenerate: entities without a shape are indexed as points.</summary>
    public void SetBounds(TKey key, Vector3 min, Vector3 max)
    {
        ObjectDisposedException.ThrowIf(_disposed, this);
        Validate(min, max);
        if (_leaves.TryGetValue(key, out var leaf))
        {
            _tree.UpdateBounds(leaf, min, max);
            return;
        }

        var bounds = new BoundingBox(min, max);
        leaf = _tree.Add(ref bounds, _pool);
        _leaves.Add(key, leaf);
        _keys.Add(key);
    }

    public bool Remove(TKey key)
    {
        ObjectDisposedException.ThrowIf(_disposed, this);
        if (!_leaves.Remove(key, out var leaf))
            return false;
        var moved = _tree.RemoveAt(leaf);
        if (moved >= 0)
        {
            var movedKey = _keys[moved];
            _keys[leaf] = movedKey;
            _leaves[movedKey] = leaf;
        }
        _keys.RemoveAt(_keys.Count - 1);
        return true;
    }

    public unsafe bool TryGetBounds(TKey key, out Vector3 min, out Vector3 max)
    {
        ObjectDisposedException.ThrowIf(_disposed, this);
        if (!_leaves.TryGetValue(key, out var leaf))
        {
            min = max = default;
            return false;
        }
        _tree.GetBoundsPointers(leaf, out var a, out var b);
        min = *a;
        max = *b;
        return true;
    }

    /// <summary>Append overlapping keys, including touching volumes. Each query owns its scratch storage.</summary>
    public void Query(Vector3 min, Vector3 max, ICollection<TKey> result)
    {
        ObjectDisposedException.ThrowIf(_disposed, this);
        ArgumentNullException.ThrowIfNull(result);
        Validate(min, max);
        if (_keys.Count == 0)
            return;

        // BEPU's public overlap traversal uses a fixed 256-entry unsafe stack.
        // Interest can contain arbitrary authoring trees: grow managed scratch
        // instead, so tree depth cannot overrun stack memory.
        var stack = ArrayPool<int>.Shared.Rent(64);
        var count = 0;
        try
        {
            Visit(_tree.Nodes[0].A);
            if (_keys.Count > 1)
                Visit(_tree.Nodes[0].B);
            while (count > 0)
            {
                var node = _tree.Nodes[stack[--count]];
                Visit(node.A);
                Visit(node.B);
            }
        }
        finally { ArrayPool<int>.Shared.Return(stack); }
        return;

        void Visit(NodeChild child)
        {
            if (!BoundingBox.Intersects(min, max, child.Min, child.Max))
                return;
            if (child.Index < 0)
            {
                result.Add(_keys[Tree.Encode(child.Index)]);
                return;
            }
            if (count == stack.Length)
            {
                var larger = ArrayPool<int>.Shared.Rent(checked(stack.Length * 2));
                stack.CopyTo(larger, 0);
                ArrayPool<int>.Shared.Return(stack);
                stack = larger;
            }
            stack[count++] = child.Index;
        }
    }

    /// <summary>Bounded incremental tree maintenance after moving leaves, before concurrent queries.</summary>
    public void Refine(int frameIndex)
    {
        ObjectDisposedException.ThrowIf(_disposed, this);
        if (frameIndex < 0)
            throw new ArgumentOutOfRangeException(nameof(frameIndex));
        _tree.RefitAndRefine(_pool, frameIndex);
    }

    private static void Validate(Vector3 min, Vector3 max)
    {
        if (!Finite(min) || !Finite(max) || min.X > max.X || min.Y > max.Y || min.Z > max.Z
            || !Finite(max - min))
            throw new ArgumentException("Index bounds must be finite and ordered.");
    }

    private static bool Finite(Vector3 value)
        => float.IsFinite(value.X) && float.IsFinite(value.Y) && float.IsFinite(value.Z);

    public void Dispose()
    {
        if (_disposed)
            return;
        _disposed = true;
        _tree.Dispose(_pool);
        _pool.Clear();
        _leaves.Clear();
        _keys.Clear();
    }
}
