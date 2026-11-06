using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Runtime.InteropServices;
using System.Threading;
using Robust.Shared.GameObjects;
using Robust.Shared.Maths;
using Robust.Shared.Spatial;

namespace Robust.Client.Graphics.Spatial;

public readonly record struct SpatialMeshChunkKey(LayerId Layer, int X, int Y);
[StructLayout(LayoutKind.Sequential)]
public readonly record struct SpatialMeshVertex(Vector3 Position, Vector3 Normal, Vector2 Uv);
public readonly record struct SpatialMeshFace(int FirstIndex, SpatialSurfaceReference Surface, EntityUid? Entity,
    ulong Revision, string MaterialProfile, SpatialFaceDirection Direction);
public readonly record struct SpatialMeshBuildLimits(int MaximumChunks, int MaximumQuads)
{
    internal void Validate()
    {
        if (MaximumChunks <= 0 || MaximumQuads <= 0 || MaximumQuads > int.MaxValue / 6)
            throw new ArgumentOutOfRangeException(nameof(MaximumQuads), "Mesh budgets must be positive and indexable.");
    }
}

/// <summary>Immutable frame-local mesh. World pose is supplied separately from the canonical Transform each draw.</summary>
public sealed class SpatialChunkMesh
{
    public SpatialGeometrySnapshot Source { get; }
    public SpatialMeshChunkKey Key { get; }
    public Box3 Bounds { get; }
    public IReadOnlyList<SpatialMeshVertex> Vertices { get; }
    public IReadOnlyList<int> Indices { get; }
    public IReadOnlyList<SpatialMeshFace> Faces { get; }
    internal ReadOnlyMemory<SpatialMeshVertex> VertexMemory { get; }
    internal ReadOnlyMemory<int> IndexMemory { get; }
    public long EstimatedBufferBytes => (long) Vertices.Count * 32 + (long) Indices.Count * sizeof(int);
    // Includes retained face metadata, excluding the separately owned snapshot.
    public long EstimatedCpuBytes => EstimatedBufferBytes + (long) Faces.Count * 64;

    private SpatialChunkMesh(SpatialGeometrySnapshot source, SpatialChunkMesh previous)
    {
        Source = source; Key = previous.Key; Bounds = previous.Bounds;
        VertexMemory = previous.VertexMemory; IndexMemory = previous.IndexMemory;
        Vertices = previous.Vertices; Indices = previous.Indices; Faces = previous.Faces;
    }

    internal SpatialChunkMesh WithSource(SpatialGeometrySnapshot source) => new(source, this);

    internal SpatialChunkMesh(SpatialGeometrySnapshot source, SpatialMeshChunkKey key,
        SpatialMeshVertex[] vertices, int[] indices, SpatialMeshFace[] faces)
    {
        Source = source; Key = key;
        VertexMemory = vertices; IndexMemory = indices;
        Vertices = Array.AsReadOnly(vertices); Indices = Array.AsReadOnly(indices); Faces = Array.AsReadOnly(faces);
        var min = vertices[0].Position;
        var max = min;
        foreach (var vertex in vertices) { min = Vector3.Min(min, vertex.Position); max = Vector3.Max(max, vertex.Position); }
        Bounds = new Box3(min, max);
    }
}

/// <summary>Build only from authorized read-only geometry. Texture/material bindings remain a content input.</summary>
public static class SpatialChunkMeshBuilder
{
    public static IReadOnlyList<SpatialChunkMesh> Build(SpatialGeometrySnapshot snapshot, float chunkSize,
        SpatialMeshBuildLimits limits, CancellationToken cancellationToken = default)
        => BuildCore(snapshot, chunkSize, limits, null, cancellationToken);

    private static IReadOnlyList<SpatialChunkMesh> BuildCore(SpatialGeometrySnapshot snapshot, float chunkSize,
        SpatialMeshBuildLimits limits, HashSet<(int X, int Y)>? dirty, CancellationToken cancellationToken)
    {
        ArgumentNullException.ThrowIfNull(snapshot);
        limits.Validate();
        if (!float.IsFinite(chunkSize) || chunkSize <= 0) throw new ArgumentOutOfRangeException(nameof(chunkSize));
        var layers = snapshot.Boundaries.Values.ToDictionary(b => b.Surface, b => b.Key.LayerId);
        foreach (var slab in snapshot.Slabs.Values) layers.Add(slab.Surface.Surface, slab.Layer);
        var chunks = new Dictionary<SpatialMeshChunkKey, List<SpatialSurfacePatch>>();
        var total = 0;
        foreach (var patch in SpatialSurfaceBuilder.Enumerate(snapshot, dirty == null ? null : IncludeSource, cancellationToken))
        {
            var axis = (int) patch.Direction / 2;
            var firstU = Cell(patch.Region.Left);
            var lastU = Last(patch.Region.Right);
            var firstV = axis == 2 ? Cell(patch.Region.Bottom) : 0;
            var lastV = axis == 2 ? Last(patch.Region.Top) : 0;
            var count = checked(((long) lastU - firstU + 1) * ((long) lastV - firstV + 1));
            if (count > limits.MaximumQuads - total) throw new InvalidOperationException("Spatial mesh quad budget exceeded.");
            for (long u = firstU; u <= lastU; u++)
            for (long v = firstV; v <= lastV; v++)
            {
                cancellationToken.ThrowIfCancellationRequested();
                var left = Math.Max(patch.Region.Left, (float) (u * chunkSize));
                var right = Math.Min(patch.Region.Right, (float) ((u + 1) * chunkSize));
                var bottom = axis == 2 ? Math.Max(patch.Region.Bottom, (float) (v * chunkSize)) : patch.Region.Bottom;
                var top = axis == 2 ? Math.Min(patch.Region.Top, (float) ((v + 1) * chunkSize)) : patch.Region.Top;
                if (right <= left || top <= bottom) continue;
                var x = axis == 0 ? Cell(patch.Plane) : (int) u;
                var y = axis == 0 ? (int) u : axis == 1 ? Cell(patch.Plane) : (int) v;
                var key = new SpatialMeshChunkKey(layers[patch.Source.Surface.Surface], x, y);
                if (dirty != null && !dirty.Contains((x, y))) continue;
                if (!chunks.TryGetValue(key, out var list))
                {
                    if (chunks.Count >= limits.MaximumChunks) throw new InvalidOperationException("Spatial mesh chunk budget exceeded.");
                    chunks.Add(key, list = new List<SpatialSurfacePatch>());
                }
                list.Add(new SpatialSurfacePatch(patch.Source, patch.Direction, patch.Plane, new Box2(left, bottom, right, top)));
                total++;
            }
        }
        var result = new List<SpatialChunkMesh>();
        foreach (var (key, patches) in chunks.OrderBy(p => p.Key.Layer.Value).ThenBy(p => p.Key.X).ThenBy(p => p.Key.Y))
        {
            cancellationToken.ThrowIfCancellationRequested();
            // Adjacent equal material profiles become contiguous draw ranges;
            // every face retains its original identity for picking/diagnostics.
            var ordered = patches.OrderBy(p => p.Source.MaterialProfile, StringComparer.Ordinal).ToArray();
            var vertices = new SpatialMeshVertex[ordered.Length * 4];
            var indices = new int[ordered.Length * 6];
            var faces = new SpatialMeshFace[ordered.Length];
            for (var i = 0; i < ordered.Length; i++)
            {
                var patch = ordered[i];
                for (var corner = 0; corner < 4; corner++)
                {
                    var point = patch.GetVertex(corner);
                    var uv = ((int) patch.Direction / 2) switch
                    {
                        0 => new Vector2(point.Y, point.Z),
                        1 => new Vector2(point.X, point.Z),
                        _ => new Vector2(point.X, point.Y)
                    };
                    vertices[i * 4 + corner] = new SpatialMeshVertex(point, patch.Normal, uv);
                }
                var first = i * 6;
                indices[first] = i * 4; indices[first + 1] = i * 4 + 1; indices[first + 2] = i * 4 + 2;
                indices[first + 3] = i * 4; indices[first + 4] = i * 4 + 2; indices[first + 5] = i * 4 + 3;
                faces[i] = new SpatialMeshFace(first, patch.Source.Surface, patch.Source.Entity, patch.Source.Revision,
                    patch.Source.MaterialProfile, patch.Direction);
            }
            result.Add(new SpatialChunkMesh(snapshot, key, vertices, indices, faces));
        }
        return result.AsReadOnly();

        bool IncludeSource(SpatialSolidPart part)
        {
            // A neighbor from another authoring layer can occlude this source.
            // Use closed XY bounds, including zero-width chunk-edge faces.
            foreach (var (x, y) in dirty!)
                if (part.Bounds.Min.X <= (double) (x + 1L) * chunkSize && part.Bounds.Max.X >= (double) x * chunkSize
                    && part.Bounds.Min.Y <= (double) (y + 1L) * chunkSize && part.Bounds.Max.Y >= (double) y * chunkSize)
                    return true;
            return false;
        }

        int Cell(float value) => CheckedIndex(Math.Floor((double) value / chunkSize));
        int Last(float value) => CheckedIndex(Math.Ceiling((double) value / chunkSize) - 1);
        static int CheckedIndex(double value)
        {
            if (!double.IsFinite(value) || value < int.MinValue || value > int.MaxValue)
                throw new ArgumentOutOfRangeException(nameof(value), "Geometry exceeds the configured chunk coordinate range.");
            return (int) value;
        }
    }

    /// <summary>Rebuild changed XY chunks and edge neighbors; retain immutable arrays in all other chunks.</summary>
    internal static IReadOnlyList<SpatialChunkMesh> BuildIncremental(SpatialGeometrySnapshot snapshot,
        SpatialGeometrySnapshot previous, IReadOnlyList<SpatialChunkMesh> previousMeshes, float chunkSize,
        SpatialMeshBuildLimits limits, CancellationToken cancellationToken)
    {
        cancellationToken.ThrowIfCancellationRequested();
        if (snapshot.OwnerFrame != previous.OwnerFrame) throw new ArgumentException("Incremental mesh owner changed.");
        var dirty = new HashSet<(int, int)>();
        var before = previous.SolidParts.GroupBy(p => p.Surface.Surface).ToDictionary(g => g.Key, g => g.ToArray());
        var after = snapshot.SolidParts.GroupBy(p => p.Surface.Surface).ToDictionary(g => g.Key, g => g.ToArray());
        foreach (var id in before.Keys.Union(after.Keys))
        {
            cancellationToken.ThrowIfCancellationRequested();
            var oldParts = before.GetValueOrDefault(id) ?? Array.Empty<SpatialSolidPart>();
            var newParts = after.GetValueOrDefault(id) ?? Array.Empty<SpatialSolidPart>();
            if (oldParts.SequenceEqual(newParts) && Layer(previous, id) == Layer(snapshot, id)) continue;
            foreach (var part in oldParts.Concat(newParts))
            {
                var minX = Math.Floor((double) part.Bounds.Min.X / chunkSize) - 1;
                var minY = Math.Floor((double) part.Bounds.Min.Y / chunkSize) - 1;
                var maxX = Math.Floor((double) part.Bounds.Max.X / chunkSize);
                var maxY = Math.Floor((double) part.Bounds.Max.Y / chunkSize);
                if (minX < int.MinValue || minY < int.MinValue || maxX > int.MaxValue || maxY > int.MaxValue
                    || (maxX - minX + 1) * (maxY - minY + 1) > limits.MaximumChunks)
                    return Build(snapshot, chunkSize, limits, cancellationToken);
                for (var x = (long) minX; x <= (long) maxX; x++)
                for (var y = (long) minY; y <= (long) maxY; y++) dirty.Add(((int) x, (int) y));
            }
            if (dirty.Count > limits.MaximumChunks) return Build(snapshot, chunkSize, limits, cancellationToken);
        }
        var rebuilt = BuildCore(snapshot, chunkSize, limits, dirty, cancellationToken);
        var retained = previousMeshes.Where(m => !dirty.Contains((m.Key.X, m.Key.Y))).Select(m => m.WithSource(snapshot));
        var result = retained.Concat(rebuilt).OrderBy(m => m.Key.Layer.Value).ThenBy(m => m.Key.X).ThenBy(m => m.Key.Y).ToArray();
        if (result.Length > limits.MaximumChunks || result.Sum(m => (long) m.Faces.Count) > limits.MaximumQuads)
            throw new InvalidOperationException("Incremental spatial mesh exceeds the complete frame budget.");
        return Array.AsReadOnly(result);

        static LayerId? Layer(SpatialGeometrySnapshot source, SurfaceId id)
        {
            if (source.Slabs.TryGetValue(id, out var slab)) return slab.Layer;
            foreach (var boundary in source.Boundaries.Values)
                if (boundary.Surface == id) return boundary.Key.LayerId;
            return null;
        }
    }
}
