using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text.Json;
using Robust.Client.Graphics;
using Robust.Client.Graphics.Spatial;
using Robust.Shared.Console;
using Robust.Shared.ContentPack;
using Robust.Shared.GameObjects;
using Robust.Shared.IoC;
using Robust.Shared.Log;
using Robust.Shared.Maths;
using Robust.Shared.Spatial;
using Robust.Shared.Utility;
using SixLabors.ImageSharp;
using SixLabors.ImageSharp.PixelFormats;
using Color = Robust.Shared.Maths.Color;

namespace Robust.Client.Console.Commands;

/// <summary>Deterministic engine graphics qualification, not a gameplay scene or content asset.</summary>
internal sealed partial class SpatialRenderProbeCommand : IConsoleCommand
{
    [Dependency] private IClydeInternal _clyde = default!;
    [Dependency] private IResourceManager _resources = default!;
    [Dependency] private IEntityManager _entities = default!;
    public string Command => "spatial_render_probe";
    public string Description => "Render the spatial qualification fixture and save PNG/JSON evidence in user data.";
    public string Help => "spatial_render_probe";

    public void Execute(IConsoleShell shell, string argStr, string[] args)
    {
        if (!_clyde.SupportsSpatialRendering) throw new NotSupportedException("Spatial graphics profile is unavailable.");
        var baseline = _clyde.GetLoadedRenderTextures().Count();
        using var checker = new Image<Rgba32>(2, 2);
        checker[0, 0] = checker[1, 1] = new Rgba32(255, 255, 255, 255);
        checker[1, 0] = checker[0, 1] = new Rgba32(255, 255, 255, 0);
        using var texture = _clyde.LoadTextureFromImage(checker, "spatial-probe-cutout");
        var scene = Scene(texture);
        var limits = new SpatialRendererLimits(32 * 1024 * 1024, 4096, 1024 * 1024);
        var captures = new Dictionary<string, Image<Rgba32>>();
        var report = new Dictionary<string, object> { ["purpose"] = "Engine render qualification; not integrated spatial gameplay", ["target_count_before"] = baseline };
        using (var renderer = _clyde.CreateSpatialRenderer(new Vector2i(800, 600), limits))
        {
            // Exercise resource replacement before capturing the final extent.
            renderer.Resize(new Vector2i(640, 480)); renderer.Resize(new Vector2i(512, 512)); renderer.Resize(new Vector2i(800, 600));
            var forward = Vector3.Normalize(new Vector3(0, 5, 1.2f) - new Vector3(8, -9, 8));
            var pitch = MathF.Atan2(forward.Z, new Vector2(forward.X, forward.Y).Length());
            var yaw = -MathF.Atan2(forward.X, forward.Y);
            var pose = new RigidTransform3(new Vector3(8, -9, 8), Quaternion.Concatenate(
                Quaternion.CreateFromAxisAngle(Vector3.UnitX, pitch), Quaternion.CreateFromAxisAngle(Vector3.UnitZ, yaw)));
            var camera = new PerspectiveCamera(pose, MathF.PI / 3, 4f / 3, 0.1f, 60);
            SpatialRenderResult result = default;
            var frames = 0;
            do
            {
                result = renderer.Render(camera, scene);
                if (result.UploadedBytes > limits.UploadBytesPerFrame)
                    throw new InvalidOperationException("Spatial upload exceeded its per-frame budget.");
                if (++frames > 1000 || result.Status == SpatialRenderStatus.BudgetExceeded)
                    throw new InvalidOperationException("Qualification fixture did not complete its bounded upload.");
            } while (result.Status != SpatialRenderStatus.Ready);
            report["upload_frames"] = frames; report["resident_mesh_bytes"] = result.ResidentMeshBytes;
            report["draw_calls"] = result.DrawCalls; report["visible_chunks"] = result.VisibleChunks;
            renderer.Target.CopyPixelsToMemory<Rgba32>(image => Capture("spatial-probe", image));
            for (var i = 0; i < 8; i++) renderer.ReloadShaders();
            report["shader_reloads"] = 8;
            var reverse = new SpatialRenderScene(scene.Instances.Reverse(), scene.Materials, scene.Lights, scene.Ambient, scene.ClearColor);
            result = renderer.Render(camera, reverse);
            if (result.Status != SpatialRenderStatus.Ready || result.UploadedBytes != 0)
                throw new InvalidOperationException("An unchanged reversed draw selection must reuse GPU buffers.");
            renderer.Target.CopyPixelsToMemory<Rgba32>(image => Capture("spatial-probe-reversed", image));
            using var composed = _clyde.CreateRenderTarget(new Vector2i(800, 600), RenderTargetColorFormat.Rgba8Srgb);
            _clyde.RenderNow(composed, handle =>
            {
                handle.DrawingHandleScreen.DrawTextureRect(renderer.Target.Texture, new UIBox2(0, 0, 800, 600));
                handle.DrawingHandleScreen.DrawRect(new UIBox2(10, 10, 30, 30), Color.Red);
                handle.DrawingHandleScreen.DrawRect(new UIBox2(40, 10, 60, 30), new Color(0f, 1f, 0f));
            });
            composed.CopyPixelsToMemory<Rgba32>(image => Capture("spatial-probe-ui", image));
        }
        var source = scene.Instances.First(i => i.Mesh.Source.OwnerFrame == new EntityUid(1)).Mesh.Source;
        var sprite = _entities.SpawnEntity("debugRotation4", Robust.Shared.Map.MapCoordinates.Nullspace);
        try
        {
            var spritePose = new RigidTransform3(new Vector3(3, 8, 3.2f), Quaternion.Identity);
            _entities.System<SharedTransformSystem>().SetLocalPose(sprite, spritePose);
            var palette = scene.Materials.ToDictionary(p => p.Key,
                p => new Color(p.Value.Color.X, p.Value.Color.Y, p.Value.Color.Z, p.Value.Color.W));
            foreach (var number in new uint[] { 1, 2 })
            {
                var layerRef = new SpatialLayerReference(source.OwnerFrame, new LayerId(number));
                var view = new TopDownSliceView(layerRef, RigidTransform3.Identity, new Vector2(0, 6), new Vector2i(800, 600), 50,
                    number == 1 ? 1.6f : 4);
                using var target = _clyde.CreateRenderTarget(new Vector2i(800, 600), RenderTargetColorFormat.Rgba8Srgb);
                _clyde.RenderNow(target, handle => view.Draw(handle, source, palette,
                    new[] { new SpatialSliceEntity(sprite, new SpatialLayerReference(source.OwnerFrame, new LayerId(2)), spritePose, Vector2.One) },
                    new[] { new SpatialSliceLine(layerRef, new Vector3(-4, 8, 3), new Vector3(-2, 8, 3), Color.Yellow) }));
                var name = $"spatial-probe-slice-{number}";
                target.CopyPixelsToMemory<Rgba32>(image => Capture(name, image));
            }
        }
        finally { _entities.DeleteEntity(sprite); }
        // Repeated independent views may not retain render targets after disposal.
        for (var i = 0; i < 8; i++)
        {
            using var renderer = _clyde.CreateSpatialRenderer(new Vector2i(64, 64), limits);
            renderer.Resize(new Vector2i(128, 96));
        }
        var after = _clyde.GetLoadedRenderTextures().Count();
        report["target_count_after"] = after;
        if (after != baseline) throw new InvalidOperationException("Spatial view disposal retained render targets.");
        shell.WriteLine("Spatial qualification submitted; PNG/JSON results are written after GPU readback.");
        return;

        void Capture(string name, Image<Rgba32> image)
        {
            using (var stream = _resources.UserData.OpenWrite(new ResPath($"/{name}.png"))) image.SaveAsPng(stream);
            captures.Add(name, image);
            if (captures.Count != 5) return;
            try
            {
                var original = captures["spatial-probe"];
                var reversed = captures["spatial-probe-reversed"];
                var maxDifference = 0;
                for (var y = 0; y < original.Height; y++)
                for (var x = 0; x < original.Width; x++)
                {
                    var a = original[x, y]; var b = reversed[x, y];
                    maxDifference = Math.Max(maxDifference, Math.Max(Math.Abs(a.R - b.R), Math.Max(Math.Abs(a.G - b.G), Math.Abs(a.B - b.B))));
                }
                var ui = captures["spatial-probe-ui"];
                var red = ui[15, 15]; var green = ui[45, 15];
                var markers = red.R > 250 && red.G < 5 && red.B < 5 && green.G > 250 && green.R < 5 && green.B < 5;
                var lower = captures["spatial-probe-slice-1"];
                var upper = captures["spatial-probe-slice-2"];
                var slices = lower[400, 300] != lower[0, 0] && upper[400, 300] == upper[0, 0]
                    && upper[500, 300] != upper[0, 0];
                var spriteFiltered = upper[550, 200] != upper[500, 200] && lower[550, 200] == lower[500, 200];
                var diagnostic = upper[250, 200] == new Rgba32(255, 255, 0, 255);
                report["reversed_max_channel_difference"] = maxDifference; report["ui_markers_correct"] = markers;
                report["slice_floor_and_opening_correct"] = slices;
                report["slice_original_sprite_filtered"] = spriteFiltered;
                report["slice_diagnostic_correct"] = diagnostic;
                report["passed"] = maxDifference <= 2 && markers && slices && spriteFiltered && diagnostic;
                _resources.UserData.WriteAllText(new ResPath("/spatial-probe.json"), JsonSerializer.Serialize(report, new JsonSerializerOptions { WriteIndented = true }));
                if (maxDifference > 2 || !markers || !slices || !spriteFiltered || !diagnostic)
                    throw new InvalidOperationException("Spatial render qualification failed; inspect spatial-probe.json.");
                Logger.InfoS("spatial.render", "Spatial render probe passed; screenshots and report saved in user data.");
            }
            finally { foreach (var captured in captures.Values) captured.Dispose(); }
        }
    }

    private static SpatialRenderScene Scene(Texture checker)
    {
        var layer = new LayerId(1);
        var instances = new List<SpatialRenderInstance>();
        var frame = new EntityUid(1);
        Add(frame, RigidTransform3.Identity, new[]
        {
            Slab(frame, 1, new Box2(-6, 0, 6, 10), 0, 0.25f, "floor"),
            new SlabSurface(new SpatialSurfaceReference(frame, new SurfaceId(2)), new LayerId(2), new Box2(-5, 3, 5, 9), 0, 0.25f,
                new[] { new Box2(-1, 4, 1, 7) }, "upper"),
            Slab(frame, 3, new Box2(-6, 9.8f, 6, 10), 5, 5, "wall"),
            Slab(frame, 4, new Box2(-3, 2, -1, 4), 1.5f, 1.5f, "box")
        });
        var standing = Quaternion.CreateFromAxisAngle(Vector3.UnitX, MathF.PI / 2);
        Add(new EntityUid(2), new RigidTransform3(new Vector3(1, 2, 1.5f), standing),
            new[] { Slab(new EntityUid(2), 1, new Box2(-2, -1, 2, 1), 0, 0.02f, "glass-a") });
        Add(new EntityUid(3), new RigidTransform3(new Vector3(1, 2.3f, 1.7f),
                Quaternion.Concatenate(standing, Quaternion.CreateFromAxisAngle(Vector3.UnitZ, -0.6f))),
            new[] { Slab(new EntityUid(3), 1, new Box2(-2, -1, 2, 1), 0, 0.02f, "glass-b") });
        Add(new EntityUid(4), new RigidTransform3(new Vector3(-3, 4, 1.5f), standing),
            new[] { Slab(new EntityUid(4), 1, new Box2(-1, -1, 1, 1), 0, 0.02f, "cutout") });
        var materials = new Dictionary<string, SpatialRenderMaterial>
        {
            ["floor"] = Material(new Vector4(0.35f, 0.38f, 0.42f, 1)),
            ["upper"] = Material(new Vector4(0.1f, 0.35f, 0.8f, 1)),
            ["wall"] = Material(new Vector4(0.45f, 0.5f, 0.55f, 1)),
            ["box"] = Material(new Vector4(0.85f, 0.25f, 0.06f, 1)),
            ["glass-a"] = Material(new Vector4(0.1f, 0.8f, 0.9f, 0.4f)) with { AlphaMode = SpatialAlphaMode.Transparent },
            ["glass-b"] = Material(new Vector4(0.85f, 0.1f, 0.6f, 0.4f)) with { AlphaMode = SpatialAlphaMode.Transparent },
            ["cutout"] = Material(new Vector4(0.8f, 0.9f, 0.15f, 1)) with { Texture = checker, AlphaMode = SpatialAlphaMode.Cutout, Repeat = new Vector2(0.5f) }
        };
        return new SpatialRenderScene(instances, materials, new[]
        {
            new SpatialRenderLight(SpatialLightKind.Point, new Vector3(0, 1, 5), 15, new Vector3(1, 0.9f, 0.7f), Vector3.Zero),
            new SpatialRenderLight(SpatialLightKind.Spot, new Vector3(5, -3, 6), 20, new Vector3(0.2f, 0.4f, 1),
                Vector3.Normalize(new Vector3(-5, 7, -6)), 0.9f, 0.65f)
        }, new Vector3(0.22f), new Vector4(0.015f, 0.02f, 0.03f, 1));
        SpatialRenderMaterial Material(Vector4 color) => new(null, color, Vector3.Zero, Vector2.One);
        SlabSurface Slab(EntityUid owner, uint id, Box2 footprint, float top, float thickness, string material)
            => new(new SpatialSurfaceReference(owner, new SurfaceId(id)), layer, footprint, top, thickness, materialProfile: material);
        void Add(EntityUid owner, RigidTransform3 pose, IEnumerable<SlabSurface> slabs)
        {
            var snapshot = new SpatialGeometrySnapshot(owner, 0,
                new Dictionary<LayerId, LayerDefinition> { [layer] = new(0, "Qualification"), [new LayerId(2)] = new(3, "Upper") },
                Array.Empty<BoundarySegment>(), slabs, Array.Empty<SpatialTileChunk>());
            foreach (var mesh in SpatialChunkMeshBuilder.Build(snapshot, 16, new SpatialMeshBuildLimits(128, 10000)))
                instances.Add(new SpatialRenderInstance(mesh, pose));
        }
    }
}
