using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Runtime.CompilerServices;
using Robust.Shared.Graphics;
using Robust.Shared.Maths;
using SixLabors.ImageSharp.PixelFormats;

namespace Robust.Client.Graphics.Spatial;

/// <summary>A resolved existing image/frame; bounds and transform are in composition pixels, tint is sRGB.</summary>
public readonly record struct SpatialSpriteLayer(Texture Texture, UIBox2 Bounds, Matrix3x2 Transform, Color Tint, UIBox2? Region = null);

/// <summary>
/// Complete appearance signature: ordered resolved frames, colors, UVs and layer
/// transforms. Direction and animation enter through the actual selected RSI
/// frames. No equipment-slot or character-type knowledge is stored here.
/// </summary>
public sealed class SpatialSpriteComposition : IEquatable<SpatialSpriteComposition>
{
    public Vector2i Size { get; }
    public IReadOnlyList<SpatialSpriteLayer> Layers { get; }
    private readonly SpatialSpriteLayer[] _layers;
    private readonly int _hash;

    public SpatialSpriteComposition(Vector2i size, IEnumerable<SpatialSpriteLayer> layers)
    {
        ArgumentNullException.ThrowIfNull(layers);
        if (size.X <= 0 || size.Y <= 0) throw new ArgumentOutOfRangeException(nameof(size));
        Size = size; _layers = layers.ToArray(); Layers = Array.AsReadOnly(_layers);
        var hash = new HashCode(); hash.Add(size);
        foreach (var layer in _layers)
        {
            var m = layer.Transform;
            if (layer.Texture == null || !ValidRect(layer.Bounds)
                || !float.IsFinite(m.M11) || !float.IsFinite(m.M12) || !float.IsFinite(m.M21)
                || !float.IsFinite(m.M22) || !float.IsFinite(m.M31) || !float.IsFinite(m.M32)
                || !float.IsFinite(layer.Tint.R) || !float.IsFinite(layer.Tint.G) || !float.IsFinite(layer.Tint.B)
                || !float.IsFinite(layer.Tint.A) || layer.Tint.R < 0 || layer.Tint.G < 0 || layer.Tint.B < 0
                || layer.Tint.A < 0 || layer.Tint.A > 1
                || layer.Region is { } r && (!ValidRect(r) || r.Left < 0 || r.Top < 0 || r.Right > layer.Texture.Width || r.Bottom > layer.Texture.Height))
                throw new ArgumentException("Invalid composition layer.");
            hash.Add(RuntimeHelpers.GetHashCode(layer.Texture)); hash.Add(layer.Bounds); hash.Add(m); hash.Add(layer.Tint); hash.Add(layer.Region);
        }
        _hash = hash.ToHashCode();
    }
    private static bool ValidRect(UIBox2 r) => float.IsFinite(r.Left) && float.IsFinite(r.Right) && float.IsFinite(r.Top)
        && float.IsFinite(r.Bottom) && float.IsFinite(r.Width) && float.IsFinite(r.Height) && r.Width > 0 && r.Height > 0;
    public bool Equals(SpatialSpriteComposition? other) => other != null && _hash == other._hash && Size == other.Size
        && _layers.AsSpan().SequenceEqual(other._layers);
    public override bool Equals(object? obj) => obj is SpatialSpriteComposition other && Equals(other);
    public override int GetHashCode() => _hash;
}

public readonly record struct SpatialSpriteCacheLimits(int MaximumEntries, long MaximumBytes, int MaximumDimension, int MaximumLayers);

/// <summary>
/// Client-thread, per-world LRU cache. Pending entries reserve the same bounded
/// bytes as resident RGBA targets. Active leases cannot be evicted. Invalidation
/// explicitly revokes even active leases; no stale clothing/resource is returned.
/// Own/dispose with the world/view owner, and invalidate on mutable-image reload.
/// </summary>
public sealed class SpatialSpriteCache : IDisposable
{
    private readonly IClyde _clyde;
    private readonly SpatialSpriteCacheLimits _limits;
    private readonly int _thread = Environment.CurrentManagedThreadId;
    private readonly Dictionary<SpatialSpriteComposition, Entry> _entries = new();
    private long _clock;
    private bool _disposed;
    public long ReservedBytes { get; private set; }
    public int Count => _entries.Count;

    public SpatialSpriteCache(IClyde clyde, SpatialSpriteCacheLimits limits)
    {
        ArgumentNullException.ThrowIfNull(clyde);
        if (limits.MaximumEntries <= 0 || limits.MaximumBytes <= 0 || limits.MaximumDimension <= 0 || limits.MaximumLayers <= 0)
            throw new ArgumentOutOfRangeException(nameof(limits));
        _clyde = clyde; _limits = limits;
    }

    /// <summary>Null means the full appearance cannot fit without evicting an active lease.</summary>
    public Lease? Request(SpatialSpriteComposition composition)
    {
        CheckThread(); ArgumentNullException.ThrowIfNull(composition);
        if (composition.Size.X > _limits.MaximumDimension || composition.Size.Y > _limits.MaximumDimension
            || composition.Layers.Count > _limits.MaximumLayers) return null;
        var pixels = (long) composition.Size.X * composition.Size.Y;
        if (pixels > int.MaxValue || pixels > _limits.MaximumBytes / 4) return null;
        var bytes = pixels * 4;
        if (!_entries.TryGetValue(composition, out var entry))
        {
            // Prove capacity first: a failed request must not destroy useful cache entries.
            var unused = _entries.Values.Where(e => e.References == 0).OrderBy(e => e.LastUse).ToArray();
            var freeBytes = _limits.MaximumBytes - ReservedBytes;
            var freeSlots = _limits.MaximumEntries - Count;
            var evictions = 0;
            while (freeBytes < bytes || freeSlots < 1)
            {
                if (evictions == unused.Length) return null;
                freeBytes += unused[evictions++].Bytes; freeSlots++;
            }
            for (var i = 0; i < evictions; i++) Remove(unused[i]);
            entry = new Entry(composition, bytes); _entries.Add(composition, entry); ReservedBytes += bytes;
        }
        entry.LastUse = ++_clock; entry.References++;
        return new Lease(this, entry);
    }

    /// <summary>
    /// Compose pending leased appearances within a target-pixel budget. The layer
    /// count is independently bounded by MaximumLayers. Ready targets are reused
    /// across frames and entities. Call before building the spatial draw selection.
    /// </summary>
    public int Pump(IRenderHandle handle, int maximumPixels)
    {
        CheckThread(); ArgumentNullException.ThrowIfNull(handle);
        if (maximumPixels <= 0) throw new ArgumentOutOfRangeException(nameof(maximumPixels));
        var spent = 0;
        foreach (var entry in _entries.Values.Where(e => e.Target == null && e.References > 0).OrderBy(e => e.LastUse))
        {
            var pixels = entry.Bytes / 4;
            if (pixels > maximumPixels - spent) continue;
            var target = _clyde.CreateRenderTarget(entry.Composition.Size, new RenderTargetFormatParameters(RenderTargetColorFormat.Rgba8Srgb, false),
                new TextureSampleParameters { Filter = false }, "spatial-appearance");
            var screen = handle.DrawingHandleScreen;
            var transform = screen.GetTransform(); var shader = screen.GetShader(); var modulate = screen.Modulate;
            try
            {
                handle.RenderInRenderTarget(target, () =>
                {
                    handle.SetScissor(null);
                    screen.UseShader(null); screen.Modulate = Color.White;
                    foreach (var layer in entry.Composition.Layers)
                    {
                        screen.SetTransform(layer.Transform);
                        screen.DrawTextureRectRegion(layer.Texture, layer.Bounds, layer.Region, layer.Tint);
                    }
                // Robust's Color.Transparent is white with zero alpha. A
                // premultiplied destination must start with zero RGB as well.
                }, new Color(0f, 0f, 0f, 0f));
                entry.Target = target; spent += (int) pixels;
            }
            catch { target.Dispose(); throw; }
            finally { screen.SetTransform(transform); screen.UseShader(shader); screen.Modulate = modulate; }
        }
        return spent;
    }

    public void Invalidate(Texture resource)
    {
        CheckThread(); ArgumentNullException.ThrowIfNull(resource);
        foreach (var entry in _entries.Values.Where(e => e.Composition.Layers.Any(l => Uses(l.Texture, resource))).ToArray()) Remove(entry);
        static bool Uses(Texture texture, Texture target)
        {
            while (true)
            {
                if (ReferenceEquals(texture, target)) return true;
                if (texture is not AtlasTexture atlas) return false;
                texture = atlas.SourceTexture;
            }
        }
    }

    public void Clear()
    {
        CheckThread();
        foreach (var entry in _entries.Values.ToArray()) Remove(entry);
    }
    private void Remove(Entry entry)
    {
        entry.Valid = false; entry.Target?.Dispose(); entry.Target = null;
        _entries.Remove(entry.Composition); ReservedBytes -= entry.Bytes;
    }
    private void CheckThread()
    {
        if (_disposed) throw new ObjectDisposedException(nameof(SpatialSpriteCache));
        if (_thread != Environment.CurrentManagedThreadId) throw new InvalidOperationException("Sprite cache requires its owning client thread.");
    }
    public void Dispose()
    {
        if (_disposed) return;
        Clear(); _disposed = true;
    }

    internal sealed class Entry(SpatialSpriteComposition composition, long bytes)
    {
        public readonly SpatialSpriteComposition Composition = composition;
        public readonly long Bytes = bytes;
        public int References;
        public long LastUse;
        public bool Valid = true;
        public IRenderTexture? Target;
    }

    public sealed class Lease : IDisposable
    {
        private SpatialSpriteCache? _owner;
        private readonly Entry _entry;
        internal Lease(SpatialSpriteCache owner, Entry entry) { _owner = owner; _entry = entry; }
        public bool IsValid => _owner != null && _entry.Valid;
        public bool IsReady => IsValid && _entry.Target != null;
        /// <summary>Premultiplied RGBA. Set SpatialRenderMaterial.PremultipliedAlpha for perspective binding.</summary>
        public Texture Texture
        {
            get
            {
                if (!IsReady) throw new InvalidOperationException("Appearance lease is pending or invalidated.");
                _owner!.CheckThread(); return _entry.Target!.Texture;
            }
        }
        /// <summary>Read back the composed premultiplied RGBA image for exports/diagnostics.</summary>
        public void CopyPixelsToMemory<T>(CopyPixelsDelegate<T> callback) where T : unmanaged, IPixel<T>
        {
            _ = Texture;
            _entry.Target!.CopyPixelsToMemory(callback);
        }
        public void Dispose()
        {
            if (_owner == null) return;
            if (_owner._thread != Environment.CurrentManagedThreadId) throw new InvalidOperationException("Sprite lease requires its owning thread.");
            _entry.References--; _owner = null;
        }
    }
}
