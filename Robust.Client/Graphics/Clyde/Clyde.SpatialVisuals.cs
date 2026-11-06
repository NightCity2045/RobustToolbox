using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using OpenToolkit.Graphics.OpenGL4;
using Robust.Client.Graphics.Spatial;

namespace Robust.Client.Graphics.Clyde;

internal sealed partial class Clyde
{
    private sealed partial class SpatialRenderer
    {
        private readonly List<SpatialVisualInstance> _visibleVisuals = new(), _requiredVisuals = new();
        private readonly HashSet<SpatialVisualShape> _requiredVisualShapes = new();
        private readonly Dictionary<SpatialVisualShape, SpatialGpuMesh> _visualMeshes = new();

        private long SelectVisuals(PerspectiveCamera camera, SpatialRenderScene scene)
        {
            _visibleVisuals.Clear(); _requiredVisuals.Clear(); _requiredVisualShapes.Clear();
            if (scene.Visuals.Count > _limits.MaximumVisuals) return 0;
            foreach (var visual in scene.Visuals)
            {
                var visible = camera.Frustum.Intersects(visual.Bounds, visual.Pose);
                if (visible) _visibleVisuals.Add(visual);
                var caster = scene.Materials[visual.MaterialProfile].AlphaMode != SpatialAlphaMode.Transparent
                    && _shadowViews.Any(v => v.Frustum.Intersects(visual.Bounds, visual.Pose));
                if (!visible && !caster) continue;
                _requiredVisuals.Add(visual); _requiredVisualShapes.Add(visual.Shape);
            }
            return _requiredVisualShapes.Sum(s => (long) (s == SpatialVisualShape.Quad ? 1 : 6) * (4 * 32 + 6 * 4));
        }

        private void RetireVisualMeshes()
        {
            foreach (var shape in _visualMeshes.Keys.Where(s => !_requiredVisualShapes.Contains(s)).ToArray())
            {
                var mesh = _visualMeshes[shape]; _residentBytes -= mesh.BufferBytes;
                mesh.Dispose(); _visualMeshes.Remove(shape);
            }
        }

        private bool UploadVisualMeshes(ref int uploaded)
        {
            var ready = true;
            foreach (var shape in _requiredVisualShapes)
            {
                if (!_visualMeshes.TryGetValue(shape, out var mesh))
                {
                    var bytes = (shape == SpatialVisualShape.Quad ? 1 : 6) * (4 * 32 + 6 * 4);
                    while (_residentBytes + bytes > _limits.MaximumMeshBytes)
                        Remove(_meshes.Keys.First(k => !_requiredKeys.Contains(k)));
                    var (vertices, indices) = VisualGeometry(shape);
                    mesh = new SpatialGpuMesh(vertices, indices);
                    _visualMeshes.Add(shape, mesh); _residentBytes += mesh.BufferBytes;
                }
                uploaded += mesh.Upload(_limits.UploadBytesPerFrame - uploaded);
                ready &= mesh.Ready;
            }
            return ready;
        }

        private int DrawVisuals(SpatialRenderScene scene, IEnumerable<SpatialVisualInstance> visuals, GLShaderProgram program, int pass)
        {
            var draws = 0;
            foreach (var visual in visuals)
            {
                var material = scene.Materials[visual.MaterialProfile];
                if ((material.AlphaMode == SpatialAlphaMode.Transparent) != (pass > 0)) continue;
                GL.BindVertexArray(_visualMeshes[visual.Shape].Vao);
                program.SetUniform("uModel", visual.Model, transpose: false);
                if (material.DoubleSided) GL.Disable(EnableCap.CullFace); else GL.Enable(EnableCap.CullFace);
                SetMaterialTexture(program, material, visual.Uv, visual.Modulate);
                if (pass >= 0)
                {
                    SetLights(scene, visual.Pose.TransformPoint(visual.Bounds.Center), visual.Bounds.Size.Length() * 0.5f);
                    program.SetUniform("uEmission", material.Emission);
                }
                GL.DrawElements(PrimitiveType.Triangles, visual.Shape == SpatialVisualShape.Quad ? 6 : 36, DrawElementsType.UnsignedInt, IntPtr.Zero);
                draws++;
            }
            return draws;
        }

        private static (SpatialMeshVertex[], int[]) VisualGeometry(SpatialVisualShape shape)
        {
            var faces = shape == SpatialVisualShape.Quad ? 1 : 6;
            var vertices = new SpatialMeshVertex[faces * 4]; var indices = new int[faces * 6];
            for (var face = 0; face < faces; face++)
            {
                var normal = shape == SpatialVisualShape.Quad ? Vector3.UnitZ : face switch
                { 0 => Vector3.UnitX, 1 => -Vector3.UnitX, 2 => Vector3.UnitY, 3 => -Vector3.UnitY, 4 => Vector3.UnitZ, _ => -Vector3.UnitZ };
                var right = MathF.Abs(normal.X) > 0.5f ? Vector3.UnitY : Vector3.UnitX;
                var up = Vector3.Cross(normal, right);
                var center = shape == SpatialVisualShape.Box ? normal * 0.5f : Vector3.Zero;
                for (var corner = 0; corner < 4; corner++)
                {
                    var x = corner is 0 or 3 ? 0 : 1;
                    var y = corner < 2 ? 0 : 1;
                    vertices[face * 4 + corner] = new SpatialMeshVertex(center + right * (x - 0.5f) + up * (y - 0.5f), normal, new Vector2(x, y));
                }
                var start = face * 6; var v = face * 4;
                indices[start] = v; indices[start + 1] = v + 1; indices[start + 2] = v + 2;
                indices[start + 3] = v; indices[start + 4] = v + 2; indices[start + 5] = v + 3;
            }
            return (vertices, indices);
        }
    }
}
