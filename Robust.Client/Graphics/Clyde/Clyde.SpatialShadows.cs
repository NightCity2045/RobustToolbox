using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using OpenToolkit.Graphics.OpenGL4;
using Robust.Client.Graphics.Spatial;
using Robust.Shared.Graphics;
using Robust.Shared.Maths;

namespace Robust.Client.Graphics.Clyde;

internal sealed partial class Clyde
{
    private sealed partial class SpatialRenderer
    {
        private RenderTexture? _shadowTarget;
        private GLShaderProgram _shadowProgram = default!;
        private readonly List<PerspectiveCamera> _shadowViews = new();
        private readonly Dictionary<SpatialRenderLight, int> _shadowBases = new();

        private void CreateShadowTarget()
        {
            var settings = _limits.Shadows;
            if (settings.MaximumLights == 0) return;
            var state = new SpatialGlState();
            try
            {
                GL.ActiveTexture(TextureUnit.Texture0);
                // Color stores projected depth; normal depth/stencil resolves the
                // nearest opaque/cutout occluder. No depth-texture extension needed.
                _shadowTarget = (RenderTexture) _owner.CreateRenderTarget(
                    new Vector2i(3 * settings.Resolution, 2 * settings.MaximumLights * settings.Resolution),
                    new RenderTargetFormatParameters(RenderTargetColorFormat.R32F, true), name: "spatial-shadows");
            }
            finally { state.Restore(); }
        }

        private void SelectShadows(PerspectiveCamera camera, SpatialRenderScene scene)
        {
            _shadowViews.Clear(); _shadowBases.Clear();
            var settings = _limits.Shadows;
            if (settings.MaximumLights == 0) return;
            // Selection changes quality only. Physics, depth and authorization do
            // not depend on whether a light fits in this decorative budget.
            foreach (var light in scene.Lights.Where(l => l.CastsShadows && l.Radius > settings.NearPlane)
                         .OrderBy(l => Vector3.DistanceSquared(l.Position, camera.Pose.Position) / ((double) l.Radius * l.Radius)))
            {
                if (_shadowBases.Count == settings.MaximumLights) break;
                if (_shadowBases.ContainsKey(light)) continue;
                _shadowBases.Add(light, _shadowViews.Count);
                for (var face = 0; face < 6; face++)
                    _shadowViews.Add(SpatialShadowProjection.Camera(light.Position, face, settings.NearPlane, light.Radius));
            }
        }

        private int DrawShadows(SpatialRenderScene scene)
        {
            if (_shadowTarget == null || _shadowViews.Count == 0) return 0;
            Bind(_shadowTarget, false);
            GL.Disable(EnableCap.Blend); GL.Enable(EnableCap.DepthTest); GL.DepthMask(true);
            GL.ClearDepth(1); GL.ClearColor(1, 1, 1, 1);
            GL.Clear(ClearBufferMask.ColorBufferBit | ClearBufferMask.DepthBufferBit);
            GL.UseProgram(_shadowProgram.Handle); _shadowProgram.SetUniform("uTexture", 0);
            var resolution = _limits.Shadows.Resolution;
            var draws = 0;
            for (var face = 0; face < _shadowViews.Count; face++)
            {
                GL.Viewport(face % 3 * resolution, face / 3 * resolution, resolution, resolution);
                var camera = _shadowViews[face];
                _shadowProgram.SetUniform("uViewProjection", camera.ViewProjection, transpose: false);
                foreach (var instance in _required)
                {
                    var mesh = instance.Mesh;
                    if (!camera.Frustum.Intersects(mesh.Bounds, instance.FramePose)) continue;
                    GL.BindVertexArray(_meshes[(mesh.Source.OwnerFrame, mesh.Key)].Vao);
                    _shadowProgram.SetUniform("uModel", instance.FramePose.ToMatrix(), transpose: false);
                    for (var i = 0; i < mesh.Faces.Count;)
                    {
                        var first = mesh.Faces[i]; var end = i + 1;
                        while (end < mesh.Faces.Count && mesh.Faces[end].MaterialProfile == first.MaterialProfile) end++;
                        var material = scene.Materials[first.MaterialProfile];
                        if (material.AlphaMode != SpatialAlphaMode.Transparent)
                        {
                            if (material.DoubleSided) GL.Disable(EnableCap.CullFace); else GL.Enable(EnableCap.CullFace);
                            SetMaterialTexture(_shadowProgram, material);
                            GL.DrawElements(PrimitiveType.Triangles, (end - i) * 6, DrawElementsType.UnsignedInt, (IntPtr) (first.FirstIndex * sizeof(int)));
                            draws++;
                        }
                        i = end;
                    }
                }
                draws += DrawVisuals(scene, _requiredVisuals.Where(v => camera.Frustum.Intersects(v.Bounds, v.Pose)), _shadowProgram, -1);
            }
            _owner.FenceRenderTarget(_shadowTarget);
            return draws;
        }

        private void SetShadowUniforms()
        {
            _forward.SetUniform("uShadowAtlas", 3);
            BindTexture(3, RenderHandle.ExtractTexture(_shadowTarget?.Texture ?? _owner.GetStockTexture(ClydeStockTexture.White), null, out _));
            if (_shadowTarget == null) return;
            var size = _shadowTarget.Size;
            _forward.SetUniform("uShadowTexel", new Vector2(1f / size.X, 1f / size.Y));
            _forward.SetUniform("uShadowBias", _limits.Shadows.DepthBias);
            var width = 1f / 3;
            var height = 1f / (2 * _limits.Shadows.MaximumLights);
            for (var i = 0; i < _shadowViews.Count; i++)
            {
                _forward.SetUniform($"uShadowMatrices[{i}]", _shadowViews[i].ViewProjection, transpose: false);
                _forward.SetUniform($"uShadowRects[{i}]", new Vector4(i % 3 * width, i / 3 * height, width, height));
            }
        }
    }
}
