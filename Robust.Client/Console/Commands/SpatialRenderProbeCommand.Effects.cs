using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using Robust.Client.Graphics;
using Robust.Client.Graphics.Spatial;
using Robust.Client.ResourceManagement;
using Robust.Shared.GameObjects;
using Robust.Shared.IoC;
using Robust.Shared.Maths;
using SixLabors.ImageSharp;
using SixLabors.ImageSharp.PixelFormats;

namespace Robust.Client.Console.Commands;

internal sealed partial class SpatialRenderProbeCommand
{
    private void CheckEffects(Action<string, Image<Rgba32>> capture, Dictionary<string, object> report)
    {
        var camera = new PerspectiveCamera(RigidTransform3.Identity, MathF.PI / 3, 1, 0.1f, 20);
        using var particles = new SpatialParticlePool(new SpatialParticleLimits(2, 10));
        var profile = new SpatialSpriteProfile(SpatialSpriteKind.DirectionalBillboard, new Vector2(1.6f, 2), new Vector2(0.5f), MathF.PI / 2);
        var red = new SpatialParticleEmission(profile, new RigidTransform3(new Vector3(0.35f, 3, 0), Quaternion.Identity),
            "particle", 4, Vector3.Zero, Vector3.Zero, profile.Size, new Vector4(1, 0, 0, 0.5f), new Vector4(1, 0, 0, 0.5f));
        particles.TryEmit(red, out _);
        particles.TryEmit(red with { Pose = new RigidTransform3(new Vector3(-0.35f, 3.5f, 0.2f), Quaternion.Identity),
            StartColor = new Vector4(0, 1, 1, 0.5f), EndColor = new Vector4(0, 1, 1, 0.5f) }, out _);
        if (particles.TryEmit(red, out _)) throw new InvalidOperationException("Particle quality budget was exceeded.");
        particles.Advance(1);
        var visuals = particles.CreateVisuals(camera, new Dictionary<EntityUid, RigidTransform3>()).ToList();
        visuals.Add(new SpatialVisualInstance(SpatialVisualShape.Box, new RigidTransform3(new Vector3(-0.65f, 2.5f, 0), Quaternion.Identity),
            new Vector3(0.4f, 0.4f, 0.8f), "occluder"));
        var materials = new Dictionary<string, SpatialRenderMaterial> {
            ["particle"] = new(null, Vector4.One, Vector3.Zero, Vector2.One, SpatialAlphaMode.Transparent, DoubleSided: true),
            ["occluder"] = new(null, new Vector4(0, 0, 1, 1), Vector3.Zero, Vector2.One) };
        var scene = new SpatialRenderScene(Array.Empty<SpatialRenderInstance>(), materials, Array.Empty<SpatialRenderLight>(),
            Vector3.One, new Vector4(0, 0, 0, 1), visuals);
        using var renderer = _clyde.CreateSpatialRenderer(new Vector2i(256, 256), new SpatialRendererLimits(4096, 4096, 256 * 256));
        var result = renderer.Render(camera, scene);
        if (result.Status != SpatialRenderStatus.Ready) throw new InvalidOperationException("Particle scene did not render.");
        renderer.Target.CopyPixelsToMemory<Rgba32>(image => capture("spatial-probe-particles", image));
        renderer.Render(camera, new SpatialRenderScene(scene.Instances, scene.Materials, scene.Lights, scene.Ambient, scene.ClearColor, visuals.AsEnumerable().Reverse()));
        renderer.Target.CopyPixelsToMemory<Rgba32>(image => capture("spatial-probe-particles-reversed", image));

        var shaderSource = IoCManager.Resolve<IResourceCache>().GetResource<ShaderSourceResource>("/Shaders/Internal/spatial-probe-effect.swsl");
        using var first = _clyde.InstanceShader(shaderSource, false, ShaderBlendMode.None);
        using var second = _clyde.InstanceShader(shaderSource, false, ShaderBlendMode.None);
        first.SetParameter("gain", new Vector3(0.5f, 1, 1)); first.SetParameter("bias", new Vector3(0.1f, 0, 0));
        second.SetParameter("gain", new Vector3(0.5f, 0.5f, 1)); second.SetParameter("bias", Vector3.Zero);
        using var effects = new SpatialScreenEffects(_clyde, new SpatialScreenEffectLimits(2, 256 * 256));
        using var scratch = _clyde.CreateRenderTarget(new Vector2i(256, 256), new RenderTargetFormatParameters(RenderTargetColorFormat.Rgba8Srgb, false));
        _clyde.RenderNow(scratch, handle =>
        {
            var passes = new[] { first, second };
            var source = renderer.Target.Texture;
            var empty = effects.Apply(handle, source, scratch.Size, Array.Empty<ShaderInstance>());
            if (!ReferenceEquals(empty.Texture, source) || effects.ResidentBytes != 0) throw new InvalidOperationException("Empty effects allocated resources.");
            var before = _clyde.GetLoadedRenderTextures().Count();
            SpatialScreenEffectResult applied = default;
            for (var i = 0; i < 8; i++)
            {
                applied = effects.Apply(handle, source, i == 3 ? new Vector2i(128, 128) : scratch.Size, passes);
                if (applied.Status != SpatialRenderStatus.Ready || applied.Passes != 2 || _clyde.GetLoadedRenderTextures().Count() != before + 2)
                    throw new InvalidOperationException("Screen passes leaked or failed across reuse/resize.");
            }
            effects.CopyPixelsToMemory<Rgba32>(image => capture("spatial-probe-effects", image));
            var oldOutput = applied.Texture!;
            try
            {
                effects.Apply(handle, new AtlasTexture(oldOutput, new UIBox2(0, 0, 256, 256)), scratch.Size, passes);
                throw new InvalidOperationException("Screen effects accepted framebuffer feedback.");
            }
            catch (ArgumentException) { }
            var rejected = effects.Apply(handle, source, scratch.Size, new[] { first, second, first });
            if (rejected.Status != SpatialRenderStatus.BudgetExceeded || rejected.Texture != null)
                throw new InvalidOperationException("Effect budget failure exposed an unfiltered image.");
            handle.DrawingHandleScreen.DrawTextureRect(oldOutput, new UIBox2(0, 0, 256, 256));
            report["screen_effect_resident_bytes"] = effects.ResidentBytes;
        });
        scratch.CopyPixelsToMemory<Rgba32>(image => capture("spatial-probe-effects-budget", image));
        particles.Advance(3);
        if (particles.Count != 0) throw new InvalidOperationException("Expired particles remained resident.");
        report["particle_budget_rejections"] = particles.RejectedByBudget;
        report["screen_effect_reuse_and_feedback_checked"] = true;
    }

    private static bool CheckEffectPixels(Dictionary<string, Image<Rgba32>> captures, Dictionary<string, object> report)
    {
        var particles = captures["spatial-probe-particles"]; var reversed = captures["spatial-probe-particles-reversed"];
        var difference = 0;
        for (var y = 0; y < particles.Height; y++)
        for (var x = 0; x < particles.Width; x++)
        {
            var a = particles[x, y]; var b = reversed[x, y];
            difference = Math.Max(difference, Math.Max(Math.Abs(a.R - b.R), Math.Max(Math.Abs(a.G - b.G), Math.Abs(a.B - b.B))));
        }
        var red = particles[164, 128]; var blue = particles[80, 128];
        var modulated = Math.Abs(red.R - 188) <= 2 && red.G == 0 && red.B == 0;
        var depth = blue == new Rgba32(0, 0, 255, 255);
        var effect = captures["spatial-probe-effects"][164, 128];
        var ordered = Math.Abs(effect.R - 116) <= 2 && effect.G == 0 && effect.B == 0;
        var failed = captures["spatial-probe-effects-budget"];
        var black = true;
        foreach (var pixel in SixLabors.ImageSharp.Advanced.AdvancedImageExtensions.GetPixelMemoryGroup(failed))
            foreach (var p in pixel.Span) black &= p == new Rgba32(0, 0, 0, 255);
        report["particle_order_difference"] = difference; report["particle_color_alpha_correct"] = modulated;
        report["particle_depth_correct"] = depth; report["ordered_screen_effects_correct"] = ordered;
        report["over_budget_effect_output_black"] = black;
        return difference <= 2 && modulated && depth && ordered && black;
    }
}
