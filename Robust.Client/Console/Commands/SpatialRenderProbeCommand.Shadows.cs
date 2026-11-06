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
    private void CheckOffscreenShadow(Action<string, Image<Rgba32>> capture, Dictionary<string, object> report)
    {
        var layer = new LayerId(1);
        var instances = new List<SpatialRenderInstance>();
        Add(new EntityUid(10), new Box2(-3, 5, 3, 11), 0, 0.25f);
        // Entirely outside the narrow camera frustum, on the segment from the
        // light to the visible floor center. It must still cast its shadow.
        Add(new EntityUid(11), new Box2(-5.25f, 3.75f, -4.75f, 4.25f), 6, 6);
        var camera = new PerspectiveCamera(new RigidTransform3(new Vector3(0, -8, 4),
            Quaternion.CreateFromAxisAngle(Vector3.UnitX, -MathF.Atan2(4, 16))), MathF.PI / 6, 1, 0.1f, 50);
        var materials = new Dictionary<string, SpatialRenderMaterial>
        {
            ["solid"] = new(null, Vector4.One, Vector3.Zero, Vector2.One)
        };
        var light = new SpatialRenderLight(SpatialLightKind.Point, new Vector3(-10, 0, 6), 30,
            Vector3.One, Vector3.Zero, CastsShadows: true);
        var scene = new SpatialRenderScene(instances, materials, new[] { light }, new Vector3(0.03f), Vector4.Zero);
        using var renderer = _clyde.CreateSpatialRenderer(new Vector2i(128, 128),
            new SpatialRendererLimits(1024 * 1024, 1024 * 1024, 1024 * 1024, new SpatialShadowSettings(1, 128, 0.05f, 0.0002f)));
        var result = renderer.Render(camera, scene);
        var visible = instances.Count(i => camera.Frustum.Intersects(i.Mesh.Bounds, i.FramePose));
        if (result.Status != SpatialRenderStatus.Ready || visible != 2 || visible >= instances.Count
            || result.ResidentMeshBytes != instances.Sum(i => i.Mesh.EstimatedBufferBytes))
            throw new InvalidOperationException("Offscreen shadow caster was omitted or fixture entered the camera frustum.");
        renderer.Target.CopyPixelsToMemory<Rgba32>(image => capture("spatial-probe-offscreen-shadow", image));
        var unshadowed = new SpatialRenderScene(instances, materials, new[] { light with { CastsShadows = false } }, scene.Ambient, scene.ClearColor);
        if (renderer.Render(camera, unshadowed).Status != SpatialRenderStatus.Ready)
            throw new InvalidOperationException("Shadow quality change interrupted visible geometry readiness.");
        renderer.Target.CopyPixelsToMemory<Rgba32>(image => capture("spatial-probe-offscreen-lit", image));
        report["offscreen_caster_loaded"] = true;

        void Add(EntityUid frame, Box2 footprint, float top, float thickness)
        {
            var snapshot = new SpatialGeometrySnapshot(frame, 0,
                new Dictionary<LayerId, LayerDefinition> { [layer] = new(0, "Shadow") },
                Array.Empty<BoundarySegment>(), new[]
                {
                    new SlabSurface(new SpatialSurfaceReference(frame, new SurfaceId(1)), layer, footprint, top, thickness, materialProfile: "solid")
                }, Array.Empty<SpatialTileChunk>());
            foreach (var mesh in SpatialChunkMeshBuilder.Build(snapshot, 16, new SpatialMeshBuildLimits(8, 100)))
                instances.Add(new SpatialRenderInstance(mesh, RigidTransform3.Identity));
        }
    }
}
