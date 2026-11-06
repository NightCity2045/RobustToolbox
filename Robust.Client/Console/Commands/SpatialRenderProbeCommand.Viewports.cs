using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using Robust.Client.Graphics;
using Robust.Client.Graphics.Spatial;
using Robust.Client.ResourceManagement;
using Robust.Client.UserInterface;
using Robust.Client.UserInterface.Controls;
using Robust.Client.UserInterface.CustomControls;
using Robust.Shared.GameObjects;
using Robust.Shared.IoC;
using Robust.Shared.Maths;
using Robust.Shared.Spatial;
using SixLabors.ImageSharp;
using SixLabors.ImageSharp.PixelFormats;
using Color = Robust.Shared.Maths.Color;

namespace Robust.Client.Console.Commands;

internal sealed partial class SpatialRenderProbeCommand
{
    private void CheckViewports(SpatialGeometrySnapshot geometry, Action<string, Image<Rgba32>> capture, Dictionary<string, object> report)
    {
        var baseline = _clyde.GetLoadedRenderTextures().Count();
        var limits = new SpatialRendererLimits(4096, 4096, 512 * 512, MaximumVisuals: 64);
        var effects = new SpatialScreenEffectLimits(1, 512 * 512);
        var root = new ProbeUiRoot(1.5f);
        var left = new SpatialViewportControl(limits, effects) { AlwaysRender = true, ResolutionScale = 0.5f };
        var right = new SpatialViewportControl(limits, effects) { AlwaysRender = true };
        root.AddChild(left); root.AddChild(right);
        root.ArrangePixel(new UIBox2i(0, 0, 800, 600));
        Layout();
        var world = new EntityUid(123);
        var framePose = new RigidTransform3(new Vector3(10, 20, 30), Quaternion.CreateFromAxisAngle(Vector3.UnitX, 0.3f));
        var scene = new SpatialRenderScene(Array.Empty<SpatialRenderInstance>(),
            new Dictionary<string, SpatialRenderMaterial> { ["red"] = new(null, new Vector4(1, 0, 0, 1), Vector3.Zero, Vector2.One) },
            Array.Empty<SpatialRenderLight>(), Vector3.One, new Vector4(0, 0, 0, 1),
            new[] { new SpatialVisualInstance(SpatialVisualShape.Box, new RigidTransform3(new Vector3(0, 3, 0), Quaternion.Identity), Vector3.One, "red") });
        var palette = geometry.SolidParts.Select(p => p.MaterialProfile).Distinct().ToDictionary(p => p, _ => new Color(0f, 1f, 0f));
        var swap = false; var denied = false; var obscure = false; var oversizedSlice = false;
        var shaderSource = IoCManager.Resolve<IResourceCache>().GetResource<ShaderSourceResource>("/Shaders/Internal/spatial-probe-effect.swsl");
        using var blackout = _clyde.InstanceShader(shaderSource, false, ShaderBlendMode.None);
        blackout.SetParameter("gain", Vector3.Zero); blackout.SetParameter("bias", Vector3.Zero);
        left.Provider = new ProbeViewProvider(size => denied ? null : Frame(size, !swap));
        right.Provider = new ProbeViewProvider(size => Frame(size, swap));
        using var target = _clyde.CreateRenderTarget(new Vector2i(800, 600), new RenderTargetFormatParameters(RenderTargetColorFormat.Rgba8Srgb, false));
        var ui = IoCManager.Resolve<IUserInterfaceManager>();
        try
        {
            Draw();
            if (left.LastStatus != SpatialRenderStatus.Ready || right.LastStatus != SpatialRenderStatus.Ready)
                throw new InvalidOperationException("Both independent viewport controls must be ready.");
            target.CopyPixelsToMemory<Rgba32>(img => capture("spatial-probe-viewport-pair", img));
            if (!left.TryCopyPixelsToMemory<Rgba32>(img => capture("spatial-probe-viewport-perspective", img))
                || !right.TryCopyPixelsToMemory<Rgba32>(img => capture("spatial-probe-viewport-slice", img)))
                throw new InvalidOperationException("A completed viewport could not be captured.");
            CheckPicking();
            var stable = _clyde.GetLoadedRenderTextures().Count();
            for (var i = 0; i < 12; i++)
            {
                swap = !swap;
                root.Scale = i % 2 == 0 ? 2 : 1.5f;
                left.UIScaleChanged(); right.UIScaleChanged();
                Layout();
                Draw();
                if (_clyde.GetLoadedRenderTextures().Count() != stable)
                    throw new InvalidOperationException("View switching or UI scaling retained targets.");
            }
            CheckPicking();
            obscure = true;
            Draw();
            if (!left.TryCopyPixelsToMemory<Rgba32>(img => capture("spatial-probe-viewport-effect", img)))
                throw new InvalidOperationException("Vision-filtered screenshot unavailable.");
            obscure = false; denied = true; left.InvalidatePresentation();
            if (left.TryGetRay(left.GlobalPixelPosition + (Vector2) left.PixelSize / 2, out _, out _)
                || left.TryCopyPixelsToMemory<Rgba32>(_ => throw new InvalidOperationException("Revoked frame was captured.")))
                throw new InvalidOperationException("Revocation retained interactive presentation.");
            Draw();
            target.CopyPixelsToMemory<Rgba32>(img => capture("spatial-probe-viewport-denied", img));
            denied = false;
            left.Visible = false;
            if (left.LastStatus != null) throw new InvalidOperationException("Hidden view retained readiness.");
            left.Visible = true; Draw();
            oversizedSlice = true; Draw();
            if (right.LastStatus != SpatialRenderStatus.BudgetExceeded
                || right.TryGetSlicePoint(right.GlobalPixelPosition + (Vector2) right.PixelSize / 2, out _, out _))
                throw new InvalidOperationException("Oversized slice selection retained interactive presentation.");
            oversizedSlice = false;
            left.ArrangePixel(new UIBox2i(20, 20, 700, 590));
            Draw(false);
            if (left.LastStatus != SpatialRenderStatus.BudgetExceeded
                || left.TryCopyPixelsToMemory<Rgba32>(_ => throw new InvalidOperationException("Over-budget frame was captured.")))
                throw new InvalidOperationException("Oversized view exposed old presentation.");
            report["viewport_switches"] = 12;
            report["viewport_ui_scale_picking_and_revocation"] = true;
        }
        finally
        {
            root.RemoveAllChildren(); left.ReleaseResources(); right.ReleaseResources();
        }
        if (_clyde.GetLoadedRenderTextures().Count() != baseline + 1)
            throw new InvalidOperationException("Detached view retained GPU resources.");

        SpatialViewFrame Frame(Vector2i size, bool perspective)
        {
            var passes = obscure ? new[] { blackout } : Array.Empty<ShaderInstance>();
            return perspective
                ? new SpatialPerspectiveFrame(world, new PerspectiveCamera(RigidTransform3.Identity, MathF.PI / 3, (float) size.X / size.Y, 0.1f, 30), scene, passes)
                : new SpatialSliceFrame(world, new TopDownSliceView(new SpatialLayerReference(geometry.OwnerFrame, new LayerId(1)), framePose,
                    new Vector2(0, 5), size, 20, 1.6f), geometry, palette, Array.Empty<SpatialSliceEntity>(),
                    oversizedSlice ? Enumerable.Repeat(new SpatialSliceLine(new SpatialLayerReference(geometry.OwnerFrame, new LayerId(1)), Vector3.Zero, Vector3.One, Color.White), 65)
                        : Array.Empty<SpatialSliceLine>(), Color.Black, passes);
        }
        void Layout()
        {
            left.ArrangePixel(new UIBox2i(20, 20, 380, 300));
            right.ArrangePixel(new UIBox2i(420, 20, 780, 300));
        }
        void Draw(bool normalResolution = true)
        {
            if (!normalResolution) left.ResolutionScale = 1;
            _clyde.RenderNow(target, handle =>
            {
                ui.RenderControl(handle, root, Vector2i.Zero);
                handle.DrawingHandleScreen.DrawRect(new UIBox2(10, 340, 30, 360), Color.Yellow);
            });
        }
        void CheckPicking()
        {
            var center = left.GlobalPixelPosition + (Vector2) left.PixelSize / 2;
            if (!left.TryGetRay(center, out var selectedWorld, out var ray) || selectedWorld != world
                || Vector3.Distance(ray.Direction, Vector3.UnitY) > 1e-5
                || !left.TryProjectWorld(new Vector3(0, 3, 0), out var pixel) || Vector2.Distance(pixel, center) > 1e-4
                || left.TryGetRay(left.GlobalPixelPosition - Vector2.One, out _, out _)
                || left.TryGetSlicePoint(center, out _, out _))
                throw new InvalidOperationException("Perspective picking diverged from displayed UI-scaled camera.");
            var rightCenter = right.GlobalPixelPosition + (Vector2) right.PixelSize / 2;
            if (!right.TryGetSlicePoint(rightCenter, out selectedWorld, out var point) || selectedWorld != world
                || Vector3.Distance(point, framePose.TransformPoint(new Vector3(0, 5, 1.6f))) > 1e-4
                || !right.TryProjectWorld(point, out pixel) || Vector2.Distance(pixel, rightCenter) > 1e-3)
                throw new InvalidOperationException("Slice picking lost the selected tilted frame/height.");
        }
    }

    private sealed class ProbeViewProvider(Func<Vector2i, SpatialViewFrame?> getFrame) : IWorldViewProvider
    {
        public SpatialViewFrame? GetFrame(Vector2i size) => getFrame(size);
    }
    private sealed class ProbeUiRoot(float scale) : UIRoot
    {
        public float Scale = scale;
        public override float UIScale => Scale;
    }

    private static bool CheckViewportPixels(Dictionary<string, Image<Rgba32>> captures, Dictionary<string, object> report)
    {
        var pair = captures["spatial-probe-viewport-pair"];
        var perspective = captures["spatial-probe-viewport-perspective"];
        var slice = captures["spatial-probe-viewport-slice"];
        var denied = captures["spatial-probe-viewport-denied"];
        var filtered = captures["spatial-probe-viewport-effect"];
        var correct = pair[200, 160] == new Rgba32(255, 0, 0, 255) && pair[600, 160] == new Rgba32(0, 255, 0, 255)
            && pair[20, 350] == new Rgba32(255, 255, 0, 255)
            && perspective.Width == 180 && perspective.Height == 140 && perspective[90, 70] == pair[200, 160]
            && slice.Width == 360 && slice.Height == 280 && slice[180, 140] == pair[600, 160]
            && denied[200, 160] == new Rgba32(0, 0, 0, 255) && denied[600, 160] == pair[600, 160];
        foreach (var memory in SixLabors.ImageSharp.Advanced.AdvancedImageExtensions.GetPixelMemoryGroup(filtered))
            foreach (var pixel in memory.Span) correct &= pixel == new Rgba32(0, 0, 0, 255);
        report["viewport_composition_and_filtered_screenshots_correct"] = correct;
        return correct;
    }
}
