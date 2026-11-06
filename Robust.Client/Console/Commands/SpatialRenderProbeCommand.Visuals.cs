using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using Robust.Client.Graphics;
using Robust.Client.Graphics.Spatial;
using Robust.Client.ResourceManagement;
using Robust.Shared.IoC;
using Robust.Shared.Maths;
using SixLabors.ImageSharp;
using SixLabors.ImageSharp.PixelFormats;
using Color = Robust.Shared.Maths.Color;

namespace Robust.Client.Console.Commands;

internal sealed partial class SpatialRenderProbeCommand
{
    private void CheckVisuals(Action<string, Image<Rgba32>> capture, Dictionary<string, object> report)
    {
        using var cache = new SpatialSpriteCache(_clyde, new SpatialSpriteCacheLimits(8, 8 * 4096, 64, 8));
        using var scratch = _clyde.CreateRenderTarget(new Vector2i(32, 32), new RenderTargetFormatParameters(RenderTargetColorFormat.Rgba8Srgb, false));
        var limits = new SpatialRendererLimits(1064, 67, 256 * 256, new SpatialShadowSettings(1, 64, 0.05f, 0.001f));
        using var renderer = _clyde.CreateSpatialRenderer(new Vector2i(256, 256), limits);
        var rsi = IoCManager.Resolve<IResourceCache>().GetResource<RSIResource>("/Textures/debugRotation.rsi").RSI;
        var state = rsi[new RSI.StateId("direction4")];
        var canonical = RigidTransform3.Identity;
        var profile = new SpatialSpriteProfile(SpatialSpriteKind.DirectionalBillboard, new Vector2(2, 2), new Vector2(0.5f, 0), MathF.PI / 3);
        var uploadedTotal = 0; var framesTotal = 0;
        foreach (var north in new[] { false, true })
        {
            var camera = new PerspectiveCamera(new RigidTransform3(new Vector3(0, north ? 4 : -4, 1),
                north ? Quaternion.CreateFromAxisAngle(Vector3.UnitZ, MathF.PI) : Quaternion.Identity), MathF.PI / 3, 1, 0.1f, 20);
            var direction = SpatialSpriteAnimation.Direction(canonical, camera, state.RsiDirections);
            var frame = state.GetFrame(direction, SpatialSpriteAnimation.Frame(state, 0));
            var composition = new SpatialSpriteComposition(new Vector2i(32, 32), new[] {
                new SpatialSpriteLayer(frame, new UIBox2(0, 0, 32, 32), Matrix3x2.Identity, Color.White),
                new SpatialSpriteLayer(Texture.White, new UIBox2(0, 0, 4, 4), Matrix3x2.Identity, Color.Blue) });
            using var lease = cache.Request(composition)!;
            _clyde.RenderNow(scratch, handle =>
            {
                var screen = handle.DrawingHandleScreen;
                var transform = Matrix3x2.CreateTranslation(10, 20);
                screen.SetTransform(transform); screen.Modulate = Color.Red;
                handle.SetScissor(new UIBox2i(100, 100, 110, 110));
                if (cache.Pump(handle, 1023) != 0 || lease.IsReady) throw new InvalidOperationException("Composition exceeded its pixel budget.");
                if (cache.Pump(handle, 1024) != 1024 || !lease.IsReady) throw new InvalidOperationException("Complete composition was not published.");
                if (screen.GetTransform() != transform || screen.Modulate != Color.Red)
                    throw new InvalidOperationException("Composition did not restore the caller's drawing state.");
                using var same = cache.Request(new SpatialSpriteComposition(composition.Size, composition.Layers))!;
                if (!ReferenceEquals(same.Texture, lease.Texture) || cache.Pump(handle, 1024) != 0)
                    throw new InvalidOperationException("Unchanged appearance allocated another target.");
                screen.SetTransform(Matrix3x2.Identity); screen.Modulate = Color.White; handle.SetScissor(null);
            });
            var visuals = new[] {
                profile.Resolve(canonical, camera, "sprite"),
                new SpatialVisualInstance(SpatialVisualShape.Box, new RigidTransform3(new Vector3(-1.2f, -0.3f, 0.3f), Quaternion.Identity),
                    new Vector3(0.6f), "box"),
                new SpatialSpriteProfile(SpatialSpriteKind.GroundSprite, new Vector2(4), new Vector2(0.5f), 0)
                    .Resolve(new RigidTransform3(new Vector3(0, 0, -0.05f), Quaternion.Identity), camera, "floor") };
            var materials = new Dictionary<string, SpatialRenderMaterial> {
                ["sprite"] = new(lease.Texture, Vector4.One, Vector3.Zero, Vector2.One, SpatialAlphaMode.Cutout, 0.5f, true, PremultipliedAlpha: true),
                ["box"] = new(null, new Vector4(0.1f, 0.25f, 0.9f, 1), Vector3.Zero, Vector2.One),
                ["floor"] = new(null, new Vector4(0.12f, 0.12f, 0.12f, 1), Vector3.Zero, Vector2.One, DoubleSided: true) };
            var scene = new SpatialRenderScene(Array.Empty<SpatialRenderInstance>(), materials,
                new[] { new SpatialRenderLight(SpatialLightKind.Point, new Vector3(0, -2, 4), 12, Vector3.One, Vector3.Zero, CastsShadows: true) },
                new Vector3(0.5f), new Vector4(0.01f, 0.01f, 0.01f, 1), visuals);
            var result = RenderComplete(camera, scene);
            if (result.VisibleVisuals != 3 || result.ResidentMeshBytes != 1064 || result.ShadowFaces != 6)
                throw new InvalidOperationException("Primitive residency/visibility/shadow selection is incorrect.");
            renderer.Target.CopyPixelsToMemory<Rgba32>(pixels => capture(north ? "spatial-probe-rsi-north" : "spatial-probe-rsi-south", pixels));
            if (!north)
            {
                RenderComplete(camera, new SpatialRenderScene(scene.Instances, scene.Materials,
                    scene.Lights.Select(l => l with { CastsShadows = false }), scene.Ambient, scene.ClearColor, scene.Visuals));
                renderer.Target.CopyPixelsToMemory<Rgba32>(pixels => capture("spatial-probe-rsi-unshadowed", pixels));
            }
        }
        if (uploadedTotal != 1064) throw new InvalidOperationException("Changing observer rebuilt shared visual geometry.");
        report["visual_upload_bytes"] = uploadedTotal; report["visual_upload_frames"] = framesTotal;
        report["directional_appearance_cache_entries"] = cache.Count;
        report["canonical_sprite_pose_unchanged"] = canonical.Equals(RigidTransform3.Identity);

        // Verify alpha composition numerically, including a translucent texture
        // rebound into the perspective path. This detects double multiplication.
        var alpha = new SpatialSpriteComposition(new Vector2i(32, 32), new[] {
            new SpatialSpriteLayer(Texture.White, new UIBox2(0, 0, 32, 32), Matrix3x2.Identity, new Color(1f, 1f, 1f, 0.5f)),
            new SpatialSpriteLayer(Texture.White, new UIBox2(0, 0, 16, 32), Matrix3x2.Identity, new Color(1f, 0f, 0f, 0.5f)) });
        using var translucent = cache.Request(alpha)!;
        _clyde.RenderNow(scratch, handle => cache.Pump(handle, 1024));
        translucent.CopyPixelsToMemory<Rgba32>(pixels => capture("spatial-probe-composition", pixels));
        var front = new PerspectiveCamera(RigidTransform3.Identity, MathF.PI / 3, 1, 0.1f, 20);
        var quad = new SpatialVisualInstance(SpatialVisualShape.Quad,
            new RigidTransform3(new Vector3(0, 3, 0), Quaternion.CreateFromAxisAngle(Vector3.UnitX, MathF.PI / 2)), new Vector3(2, 2, 1), "alpha");
        var alphaScene = new SpatialRenderScene(Array.Empty<SpatialRenderInstance>(),
            new Dictionary<string, SpatialRenderMaterial> { ["alpha"] = new(translucent.Texture, Vector4.One, Vector3.Zero, Vector2.One,
                SpatialAlphaMode.Transparent, DoubleSided: true, PremultipliedAlpha: true) },
            Array.Empty<SpatialRenderLight>(), Vector3.One, new Vector4(0, 0, 0, 1), new[] { quad });
        RenderComplete(front, alphaScene);
        renderer.Target.CopyPixelsToMemory<Rgba32>(pixels => capture("spatial-probe-composition-perspective", pixels));
        cache.Invalidate(Texture.White);
        if (translucent.IsValid || cache.ReservedBytes != 0) throw new InvalidOperationException("Source invalidation retained stale appearances.");
        report["appearance_invalidation_revokes_leases"] = true;
        using var failed = _clyde.CreateRenderTarget(new Vector2i(32, 32), new RenderTargetFormatParameters(RenderTargetColorFormat.Rgba8Srgb, false));
        _clyde.RenderNow(scratch, handle =>
        {
            var screen = handle.DrawingHandleScreen;
            screen.DrawRect(new UIBox2(0, 0, 32, 32), new Color(0f, 1f, 0f));
            try
            {
                handle.RenderInRenderTarget(failed, () =>
                {
                    screen.DrawRect(new UIBox2(0, 0, 32, 32), Color.Red);
                    throw new ProbeDrawException();
                }, Color.Transparent);
            }
            catch (ProbeDrawException) { }
            screen.DrawRect(new UIBox2(0, 0, 8, 8), Color.Blue);
        });
        scratch.CopyPixelsToMemory<Rgba32>(pixels => capture("spatial-probe-composition-recovery", pixels));
        return;

        SpatialRenderResult RenderComplete(PerspectiveCamera camera, SpatialRenderScene scene)
        {
            SpatialRenderResult result;
            var attempts = 0;
            do
            {
                result = renderer.Render(camera, scene); uploadedTotal += result.UploadedBytes; framesTotal++;
                if (result.UploadedBytes > 67 || result.Status == SpatialRenderStatus.BudgetExceeded || ++attempts > 100)
                    throw new InvalidOperationException("Visual mesh upload did not respect its limits.");
            } while (result.Status != SpatialRenderStatus.Ready);
            return result;
        }
    }

    private static bool CheckVisualPixels(Dictionary<string, Image<Rgba32>> captures, Dictionary<string, object> report)
    {
        var composition = captures["spatial-probe-composition"];
        var right = composition[24, 24]; var left = composition[8, 24];
        var alphaCorrect = Near(right.R, 188) && Near(right.G, 188) && Near(right.B, 188) && Near(right.A, 128)
            && Near(left.R, 225) && Near(left.G, 137) && Near(left.B, 137) && Near(left.A, 191);
        var perspective = captures["spatial-probe-composition-perspective"][164, 128];
        var unpremultiplied = Near(perspective.R, 188) && Near(perspective.G, 188) && Near(perspective.B, 188);
        var south = captures["spatial-probe-rsi-south"]; var north = captures["spatial-probe-rsi-north"];
        var different = 0;
        for (var y = 64; y < 192; y++)
        for (var x = 64; x < 192; x++)
            if (south[x, y] != north[x, y]) different++;
        var unshadowed = captures["spatial-probe-rsi-unshadowed"];
        var shadowPixels = 0;
        for (var y = 0; y < south.Height; y++)
        for (var x = 0; x < south.Width; x++)
            if (south[x, y].R + south[x, y].G + south[x, y].B + 5 < unshadowed[x, y].R + unshadowed[x, y].G + unshadowed[x, y].B) shadowPixels++;
        report["composition_alpha_correct"] = alphaCorrect;
        report["composition_perspective_alpha_correct"] = unpremultiplied;
        report["observer_rsi_different_pixels"] = different;
        report["visual_shadowed_pixels"] = shadowPixels;
        var recovery = captures["spatial-probe-composition-recovery"];
        var restored = recovery[20, 20] == new Rgba32(0, 255, 0, 255) && recovery[4, 4] == new Rgba32(0, 0, 255, 255);
        report["failed_composition_restores_target"] = restored;
        return alphaCorrect && unpremultiplied && different > 100 && shadowPixels > 100 && restored;
        static bool Near(byte value, int expected) => Math.Abs(value - expected) <= 2;
    }

    private sealed class ProbeDrawException : Exception;
}
