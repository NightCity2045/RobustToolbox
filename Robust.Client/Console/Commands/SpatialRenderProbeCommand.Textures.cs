using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using Robust.Client.Graphics;
using Robust.Client.Graphics.Spatial;
using Robust.Shared.GameObjects;
using Robust.Shared.Maths;
using Robust.Shared.Spatial;
using SixLabors.ImageSharp;
using SixLabors.ImageSharp.PixelFormats;

namespace Robust.Client.Console.Commands;

internal sealed partial class SpatialRenderProbeCommand
{
    private void CheckMaterialMips(Action<string, Image<Rgba32>> capture, Dictionary<string, object> report)
    {
        var baseline = _clyde.GetLoadedTextures().Count();
        using var atlas = new Image<Rgba32>(96, 32, new Rgba32(255, 0, 255, 255));
        for (var y = 0; y < 32; y++)
        for (var x = 32; x < 64; x++) atlas[x, y] = (x + y) % 2 == 0 ? new Rgba32(0, 0, 0, 255) : new Rgba32(255, 255, 255, 255);
        var chain = SpatialTextureMipChain.Build(atlas, new UIBox2i(32, 0, 64, 32), new SpatialTextureBuildLimits(64, 16384));
        using (var upload = _clyde.CreateSpatialTextureUpload(chain, chain.Bytes, "spatial-probe-mips"))
        {
            var hidden = false;
            try { _ = upload.Texture; } catch (InvalidOperationException) { hidden = true; }
            if (!hidden) throw new InvalidOperationException("An incomplete material texture became visible.");
            var calls = 0;
            while (!upload.IsReady)
            {
                var bytes = upload.Upload(13); // deliberately splits RGBA scanlines
                if (bytes <= 0 || bytes > 13 || ++calls > 1000)
                    throw new InvalidOperationException("Material upload failed to respect its byte budget.");
            }
            if (upload.UploadedBytes != chain.Bytes || upload.AllocatedBytes != chain.Bytes)
                throw new InvalidOperationException("Material pyramid byte accounting is inconsistent.");
            report["material_upload_calls"] = calls; report["material_mip_bytes"] = chain.Bytes;
            report["pending_material_hidden"] = hidden;

            var frame = new EntityUid(20); var layer = new LayerId(1);
            var snapshot = new SpatialGeometrySnapshot(frame, 0,
                new Dictionary<LayerId, LayerDefinition> { [layer] = new(0, "Material") }, Array.Empty<BoundarySegment>(),
                new[] { new SlabSurface(new SpatialSurfaceReference(frame, new SurfaceId(1)), layer, new Box2(-1, -1, 1, 1), 0, 0.02f, materialProfile: "checker") },
                Array.Empty<SpatialTileChunk>());
            var pose = new RigidTransform3(new Vector3(0, 3, 0), Quaternion.CreateFromAxisAngle(Vector3.UnitX, MathF.PI / 2));
            var instances = SpatialChunkMeshBuilder.Build(snapshot, 16, new SpatialMeshBuildLimits(16, 100))
                .Select(mesh => new SpatialRenderInstance(mesh, pose)).ToArray();
            var camera = new PerspectiveCamera(RigidTransform3.Identity, MathF.PI / 3, 1, 0.1f, 20);
            using var renderer = _clyde.CreateSpatialRenderer(new Vector2i(256, 256), new SpatialRendererLimits(1024 * 1024, 1024 * 1024, 256 * 256));
            foreach (var near in new[] { true, false })
            {
                var materials = new Dictionary<string, SpatialRenderMaterial>
                {
                    ["checker"] = new(upload.Texture, Vector4.One, Vector3.Zero, new Vector2(near ? 0.125f : 8), Sampling: SpatialTextureSampling.PixelMip)
                };
                var scene = new SpatialRenderScene(instances, materials, Array.Empty<SpatialRenderLight>(), Vector3.One, Vector4.Zero);
                if (renderer.Render(camera, scene).Status != SpatialRenderStatus.Ready)
                    throw new InvalidOperationException("Material sampling fixture was not ready.");
                var name = near ? "spatial-probe-material-near" : "spatial-probe-material-far";
                renderer.Target.CopyPixelsToMemory<Rgba32>(image => capture(name, image));
            }
        }
        using (var cancelled = _clyde.CreateSpatialTextureUpload(chain, chain.Bytes, "spatial-probe-cancelled-mips"))
        {
            cancelled.Upload(4);
            if (cancelled.IsReady) throw new InvalidOperationException("Partial mip fixture unexpectedly completed.");
        }
        var after = _clyde.GetLoadedTextures().Count();
        report["material_texture_count_before"] = baseline; report["material_texture_count_after"] = after;
        if (after != baseline) throw new InvalidOperationException("Complete/cancelled material upload retained GPU textures.");
    }
}
