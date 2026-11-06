using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using Robust.Shared.Graphics;
using Robust.Shared.Maths;
using SixLabors.ImageSharp.PixelFormats;

namespace Robust.Client.Graphics.Spatial;

public readonly record struct SpatialScreenEffectLimits(int MaximumPasses, int MaximumPixels);
public readonly record struct SpatialScreenEffectResult(SpatialRenderStatus Status, Texture? Texture, int Passes);

/// <summary>
/// Per-view bounded post-processing, before ordinary UI composition. Existing
/// content-owned ShaderInstances read the preceding image through TEXTURE/UV.
/// Exactly two reusable targets; no framebuffer feedback, no per-frame targets.
/// Failure returns no unfiltered image, and clears previously exposed owned output.
/// Does not choose blindness/vision policy, effect parameters or observer rights.
/// </summary>
public sealed class SpatialScreenEffects : IDisposable
{
    private readonly IClyde _clyde;
    private readonly SpatialScreenEffectLimits _limits;
    private readonly int _thread = Environment.CurrentManagedThreadId;
    private IRenderTexture? _first, _second, _last;
    private bool _disposed;
    public long ResidentBytes => _first == null ? 0 : (long) _first.Size.X * _first.Size.Y * 8;

    public SpatialScreenEffects(IClyde clyde, SpatialScreenEffectLimits limits)
    {
        ArgumentNullException.ThrowIfNull(clyde);
        if (limits.MaximumPasses < 0 || limits.MaximumPixels <= 0) throw new ArgumentOutOfRangeException(nameof(limits));
        _clyde = clyde; _limits = limits;
    }

    public SpatialScreenEffectResult Apply(IRenderHandle handle, Texture source, Vector2i size, IReadOnlyList<ShaderInstance> passes)
    {
        CheckThread(); ArgumentNullException.ThrowIfNull(handle); ArgumentNullException.ThrowIfNull(source); ArgumentNullException.ThrowIfNull(passes);
        if (size.X <= 0 || size.Y <= 0) throw new ArgumentOutOfRangeException(nameof(size));
        var root = source;
        while (root is AtlasTexture atlas) root = atlas.SourceTexture;
        if (ReferenceEquals(root, _first?.Texture) || ReferenceEquals(root, _second?.Texture))
            throw new ArgumentException("A screen effect chain cannot consume its own previous output.");
        _last = null;
        if (passes.Count > _limits.MaximumPasses || (long) size.X * size.Y > _limits.MaximumPixels)
        {
            ClearOutputs(handle);
            return new SpatialScreenEffectResult(SpatialRenderStatus.BudgetExceeded, null, 0);
        }
        if (passes.Any(p => p == null || p.Disposed || p.StencilTestEnabled))
        {
            ClearOutputs(handle);
            throw new ArgumentException("Screen effects need live shaders without stencil requirements.");
        }
        if (passes.Count == 0) return new SpatialScreenEffectResult(SpatialRenderStatus.Ready, source, 0);
        var screen = handle.DrawingHandleScreen;
        var transform = screen.GetTransform(); var oldShader = screen.GetShader(); var modulate = screen.Modulate;
        try
        {
            Resize(size);
            for (var i = 0; i < passes.Count; i++)
            {
                var target = i % 2 == 0 ? _first! : _second!;
                var shader = passes[i]; var input = source;
                handle.RenderInRenderTarget(target, () =>
                {
                    handle.SetScissor(null); screen.SetTransform(Matrix3x2.Identity); screen.Modulate = Color.White;
                    screen.UseShader(shader);
                    screen.DrawTextureRect(input, new UIBox2(0, 0, size.X, size.Y));
                }, new Color(0f, 0f, 0f, 1f));
                source = target.Texture; _last = target;
            }
            return new SpatialScreenEffectResult(SpatialRenderStatus.Ready, source, passes.Count);
        }
        catch { _last = null; ClearOutputs(handle); throw; }
        finally { screen.SetTransform(transform); screen.UseShader(oldShader); screen.Modulate = modulate; }
    }

    private void Resize(Vector2i size)
    {
        if (_first?.Size == size) return;
        IRenderTexture? first = null, second = null;
        try
        {
            var format = new RenderTargetFormatParameters(RenderTargetColorFormat.Rgba8Srgb, false);
            var samples = new TextureSampleParameters { Filter = false };
            first = _clyde.CreateRenderTarget(size, format, samples, "spatial-screen-a");
            second = _clyde.CreateRenderTarget(size, format, samples, "spatial-screen-b");
        }
        catch { first?.Dispose(); second?.Dispose(); throw; }
        _first?.Dispose(); _second?.Dispose(); _first = first; _second = second;
    }

    private void ClearOutputs(IRenderHandle handle)
    {
        if (_first != null) handle.RenderInRenderTarget(_first, () => { }, new Color(0f, 0f, 0f, 1f));
        if (_second != null) handle.RenderInRenderTarget(_second, () => { }, new Color(0f, 0f, 0f, 1f));
    }

    public void CopyPixelsToMemory<T>(CopyPixelsDelegate<T> callback) where T : unmanaged, IPixel<T>
    {
        CheckThread();
        if (_last == null) throw new InvalidOperationException("No completed screen-effect output is available.");
        _last.CopyPixelsToMemory(callback);
    }
    private void CheckThread()
    {
        if (_disposed) throw new ObjectDisposedException(nameof(SpatialScreenEffects));
        if (_thread != Environment.CurrentManagedThreadId) throw new InvalidOperationException("Screen effects require the owning client thread.");
    }
    public void Dispose()
    {
        if (_disposed) return;
        CheckThread(); _disposed = true; _first?.Dispose(); _second?.Dispose(); _first = _second = _last = null;
    }
}
