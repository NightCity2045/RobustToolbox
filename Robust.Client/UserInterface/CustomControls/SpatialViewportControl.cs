using System;
using System.Numerics;
using Robust.Client.Graphics;
using Robust.Client.Graphics.Spatial;
using Robust.Shared.GameObjects;
using Robust.Shared.Graphics;
using Robust.Shared.IoC;
using Robust.Shared.Maths;
using SixLabors.ImageSharp.PixelFormats;

namespace Robust.Client.UserInterface.CustomControls;

/// <summary>
/// Ordinary UI control hosting a perspective or authoring slice of the same world.
/// Each control owns bounded rendering resources, never entities or subscriptions.
/// Content owns camera/input/permissions and supplies a fresh authorized frame.
/// XYZ picking deliberately does not implement the planar IViewportControl API.
/// </summary>
public sealed class SpatialViewportControl : Control
{
    private readonly IClyde _clyde;
    private readonly SpatialRendererLimits _limits;
    private readonly SpatialScreenEffectLimits _effectLimits;
    private ISpatialRenderer? _renderer;
    private IRenderTexture? _slice;
    private SpatialScreenEffects? _effects;
    private IWorldViewProvider? _provider;
    private SpatialViewFrame? _presented;
    private Vector2i _presentedSize;
    private float _resolutionScale = 1;
    public SpatialRenderStatus? LastStatus { get; private set; }

    public IWorldViewProvider? Provider
    {
        get => _provider;
        set { if (ReferenceEquals(_provider, value)) return; ReleaseResources(); _provider = value; }
    }

    /// <summary>Scalar physical resolution multiplier; does not change UI size or picking.</summary>
    public float ResolutionScale
    {
        get => _resolutionScale;
        set
        {
            if (!float.IsFinite(value) || value <= 0 || value > 1) throw new ArgumentOutOfRangeException(nameof(value));
            if (_resolutionScale == value) return;
            _resolutionScale = value; InvalidatePresentation();
        }
    }

    public SpatialViewportControl(SpatialRendererLimits limits, SpatialScreenEffectLimits effectLimits)
    {
        limits.Validate();
        if (effectLimits.MaximumPasses < 0 || effectLimits.MaximumPixels <= 0) throw new ArgumentOutOfRangeException(nameof(effectLimits));
        _clyde = IoCManager.Resolve<IClyde>(); _limits = limits; _effectLimits = effectLimits;
        MouseFilter = MouseFilterMode.Stop;
        RectClipContent = true;
    }

    /// <summary>Call immediately when content changes rights/readiness, before accepting further input or screenshots.</summary>
    public void InvalidatePresentation() { _presented = null; LastStatus = null; }

    protected internal override void Draw(IRenderHandle handle)
    {
        InvalidatePresentation();
        var screen = handle.DrawingHandleScreen;
        var shader = screen.GetShader();
        try
        {
            screen.UseShader(null);
            screen.DrawRect(PixelSizeBox, Color.Black);
            if (!VisibleInTree || PixelWidth <= 0 || PixelHeight <= 0 || _provider == null)
            {
                ReleaseResources(); return;
            }
            var size = Vector2i.ComponentMax(Vector2i.One, (Vector2i) (PixelSize * _resolutionScale));
            if ((long) size.X * size.Y > _limits.MaximumPixels || (long) size.X * size.Y > _effectLimits.MaximumPixels)
            {
                ReleaseResources(); LastStatus = SpatialRenderStatus.BudgetExceeded; return;
            }
            var frame = _provider.GetFrame(size);
            if (frame == null) { ReleaseResources(); return; }
            Texture texture;
            switch (frame)
            {
                case SpatialPerspectiveFrame perspective:
                    if (MathF.Abs(perspective.Camera.AspectRatio - (float) size.X / size.Y) > 1e-5f)
                        throw new ArgumentException("Provider camera aspect must match the requested render size.");
                    _slice?.Dispose(); _slice = null;
                    _renderer ??= _clyde.CreateSpatialRenderer(size, _limits);
                    _renderer.Resize(size);
                    LastStatus = _renderer.Render(perspective.Camera, perspective.Scene).Status;
                    if (LastStatus != SpatialRenderStatus.Ready) return;
                    texture = _renderer.Target.Texture;
                    break;
                case SpatialSliceFrame slice:
                    if (slice.View.Size != size) throw new ArgumentException("Provider slice must match the requested render size.");
                    if ((long) slice.Geometry.SolidParts.Count + slice.Entities.Count + slice.Diagnostics.Count > _limits.MaximumVisuals)
                    {
                        ReleaseResources(); LastStatus = SpatialRenderStatus.BudgetExceeded; return;
                    }
                    _renderer?.Dispose(); _renderer = null;
                    if (_slice?.Size != size)
                    {
                        var target = _clyde.CreateRenderTarget(size, new RenderTargetFormatParameters(RenderTargetColorFormat.Rgba8Srgb, false));
                        _slice?.Dispose(); _slice = target;
                    }
                    handle.RenderInRenderTarget(_slice, () =>
                    {
                        handle.SetScissor(null);
                        var modulation = screen.Modulate;
                        try
                        {
                            screen.Modulate = Color.White;
                            slice.View.Draw(handle, slice.Geometry, slice.Palette, slice.Entities, slice.Diagnostics);
                        }
                        finally { screen.Modulate = modulation; }
                    }, slice.ClearColor);
                    texture = _slice.Texture;
                    break;
                default: throw new ArgumentException("Unknown spatial view descriptor.");
            }
            if (frame.Effects.Count != 0)
            {
                _effects ??= new SpatialScreenEffects(_clyde, _effectLimits);
                var result = _effects.Apply(handle, texture, size, frame.Effects);
                LastStatus = result.Status;
                if (result.Status != SpatialRenderStatus.Ready) return;
                texture = result.Texture!;
            }
            else { _effects?.Dispose(); _effects = null; }
            screen.DrawTextureRect(texture, PixelSizeBox);
            _presented = frame; _presentedSize = PixelSize; LastStatus = SpatialRenderStatus.Ready;
        }
        catch { InvalidatePresentation(); throw; }
        finally { screen.UseShader(shader); }
    }

    private bool NormalizeScreen(Vector2 absolutePixel, out Vector2 normalized)
    {
        normalized = default;
        if (_presented == null || !VisibleInTree || _presentedSize != PixelSize) return false;
        var local = absolutePixel - GlobalPixelPosition;
        if (!float.IsFinite(local.X) || !float.IsFinite(local.Y) || local.X < 0 || local.Y < 0
            || local.X >= PixelWidth || local.Y >= PixelHeight) return false;
        normalized = local / (Vector2) PixelSize; return true;
    }

    public bool TryGetRay(Vector2 absolutePixel, out EntityUid world, out SpatialCameraRay ray)
    {
        world = default; ray = default;
        if (!NormalizeScreen(absolutePixel, out var normalized) || _presented is not SpatialPerspectiveFrame frame) return false;
        world = frame.World; ray = frame.Camera.GetRay(normalized); return true;
    }

    public bool TryGetSlicePoint(Vector2 absolutePixel, out EntityUid world, out Vector3 position)
    {
        world = default; position = default;
        if (!NormalizeScreen(absolutePixel, out var normalized) || _presented is not SpatialSliceFrame frame) return false;
        var point = frame.View.UnprojectWorld(normalized * frame.View.Size);
        if (!float.IsFinite(point.X) || !float.IsFinite(point.Y) || !float.IsFinite(point.Z)) return false;
        world = frame.World; position = point; return true;
    }

    public bool TryProjectWorld(Vector3 position, out Vector2 absolutePixel)
    {
        absolutePixel = default;
        if (_presented == null || !VisibleInTree || _presentedSize != PixelSize
            || !float.IsFinite(position.X) || !float.IsFinite(position.Y) || !float.IsFinite(position.Z)) return false;
        Vector2 normalized;
        if (_presented is SpatialPerspectiveFrame perspective)
        {
            if (!perspective.Camera.TryProject(position, out var screen) || screen.Z < -1 || screen.Z > 1) return false;
            normalized = new Vector2(screen.X, screen.Y);
        }
        else if (_presented is SpatialSliceFrame slice) normalized = slice.View.ProjectWorld(position) / (Vector2) slice.View.Size;
        else return false;
        if (!float.IsFinite(normalized.X) || !float.IsFinite(normalized.Y)
            || normalized.X < 0 || normalized.Y < 0 || normalized.X >= 1 || normalized.Y >= 1) return false;
        absolutePixel = normalized * PixelSize + GlobalPixelPosition; return true;
    }

    /// <summary>Captures the completed view with vision effects, excluding surrounding UI. False while unavailable.</summary>
    public bool TryCopyPixelsToMemory<T>(CopyPixelsDelegate<T> callback) where T : unmanaged, IPixel<T>
    {
        if (_presented == null || !VisibleInTree || _presentedSize != PixelSize) return false;
        if (_effects != null) _effects.CopyPixelsToMemory(callback);
        else if (_presented is SpatialPerspectiveFrame) _renderer!.Target.CopyPixelsToMemory(callback);
        else _slice!.CopyPixelsToMemory(callback);
        return true;
    }

    public void ReleaseResources()
    {
        InvalidatePresentation();
        _renderer?.Dispose(); _renderer = null;
        _slice?.Dispose(); _slice = null;
        _effects?.Dispose(); _effects = null;
    }

    protected override void Resized() { base.Resized(); InvalidatePresentation(); }
    protected internal override void UIScaleChanged() { base.UIScaleChanged(); InvalidatePresentation(); }
    protected override void VisibilityChanged(bool newVisible) { base.VisibilityChanged(newVisible); if (!newVisible) ReleaseResources(); }
    protected override void ExitedTree() { ReleaseResources(); base.ExitedTree(); }
    [Obsolete("Controls should only be removed from UI tree instead of being disposed")]
    protected override void Dispose(bool disposing) { if (disposing) ReleaseResources(); base.Dispose(disposing); }
}
