using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Runtime.InteropServices;
using OpenToolkit.Graphics.OpenGL4;
using Robust.Client.Graphics.Spatial;
using Robust.Shared.GameObjects;
using Robust.Shared.Maths;

namespace Robust.Client.Graphics.Clyde;

internal sealed partial class Clyde
{
    private readonly List<SpatialRenderer> _spatialRenderers = new();
    // Desktop core/compat GL with sampler objects is the initial spatial profile.
    // GLES requires separate qualification of 32-bit floating-point blending.
    public bool SupportsSpatialRendering => IsInitialized && !_isGLES && _hasGLVertexArrayObject
        && _hasGLSamplerObjects && _hasGLFloatFramebuffers && _hasGLSrgb;

    public ISpatialRenderer CreateSpatialRenderer(Vector2i size, SpatialRendererLimits limits)
    {
        if (!SupportsSpatialRendering) throw new NotSupportedException("The spatial renderer requires the desktop floating-point GL profile.");
        if (!IsMainThread()) throw new InvalidOperationException("Graphics resources require the Clyde thread.");
        limits.Validate();
        var renderer = new SpatialRenderer(this, size, limits);
        _spatialRenderers.Add(renderer);
        return renderer;
    }

    private sealed partial class SpatialRenderer : ISpatialRenderer
    {
        private readonly Clyde _owner;
        private readonly SpatialRendererLimits _limits;
        private readonly Dictionary<(EntityUid, SpatialMeshChunkKey), SpatialGpuMesh> _meshes = new();
        private readonly List<SpatialRenderInstance> _visible = new();
        private readonly List<SpatialRenderInstance> _required = new();
        private readonly HashSet<(EntityUid, SpatialMeshChunkKey)> _requiredKeys = new();
        private readonly HashSet<(EntityUid, SpatialMeshChunkKey)> _sceneKeys = new();
        private RenderTexture _opaque = default!, _accumulation = default!, _revealage = default!, _output = default!;
        private GLShaderProgram _forward = default!, _composite = default!;
        private int _sampler, _mipSampler, _fullscreenVao;
        private long _residentBytes;
        private bool _disposed;
        public IRenderTexture Target => _output;

        public SpatialRenderer(Clyde owner, Vector2i size, SpatialRendererLimits limits)
        {
            _owner = owner; _limits = limits;
            var state = new SpatialGlState();
            try
            {
                ReloadShaders();
                _fullscreenVao = GL.GenVertexArray();
                _sampler = GL.GenSampler();
                GL.SamplerParameter(_sampler, SamplerParameterName.TextureMinFilter, (int) TextureMinFilter.Nearest);
                GL.SamplerParameter(_sampler, SamplerParameterName.TextureMagFilter, (int) TextureMagFilter.Nearest);
                GL.SamplerParameter(_sampler, SamplerParameterName.TextureWrapS, (int) TextureWrapMode.ClampToEdge);
                GL.SamplerParameter(_sampler, SamplerParameterName.TextureWrapT, (int) TextureWrapMode.ClampToEdge);
                _mipSampler = GL.GenSampler();
                GL.SamplerParameter(_mipSampler, SamplerParameterName.TextureMinFilter, (int) TextureMinFilter.NearestMipmapLinear);
                GL.SamplerParameter(_mipSampler, SamplerParameterName.TextureMagFilter, (int) TextureMagFilter.Nearest);
                GL.SamplerParameter(_mipSampler, SamplerParameterName.TextureWrapS, (int) TextureWrapMode.Repeat);
                GL.SamplerParameter(_mipSampler, SamplerParameterName.TextureWrapT, (int) TextureWrapMode.Repeat);
                Resize(size);
                CreateShadowTarget();
                owner.CheckGlError();
            }
            catch { Dispose(); throw; }
            finally { state.Restore(); }
        }

        private GLShaderProgram Program(string name, (string, uint)[] attributes)
            => _owner._compileProgram(_owner._resManager.ContentFileReadAllText($"/Shaders/Internal/{name}.vert"),
                _owner._resManager.ContentFileReadAllText($"/Shaders/Internal/{name}.frag"), attributes, name, includeLib: false);

        public void ReloadShaders()
        {
            CheckThread();
            _owner.FlushRenderQueue();
            var state = new SpatialGlState();
            GLShaderProgram? forward = null, composite = null, shadow = null;
            try
            {
                forward = Program("spatial-forward", new[] { ("aPosition", 0u), ("aNormal", 1u), ("aUv", 2u) });
                composite = Program("spatial-composite", Array.Empty<(string, uint)>());
                shadow = Program("spatial-shadow", new[] { ("aPosition", 0u), ("aUv", 2u) });
                var oldForward = _forward; var oldComposite = _composite; var oldShadow = _shadowProgram;
                (_forward, _composite) = (forward, composite);
                _shadowProgram = shadow;
                forward = composite = shadow = null;
                oldForward?.Delete(); oldComposite?.Delete(); oldShadow?.Delete();
            }
            finally
            {
                forward?.Delete(); composite?.Delete(); shadow?.Delete();
                state.Restore();
            }
        }

        public void Resize(Vector2i size)
        {
            CheckThread();
            if (size.X <= 0 || size.Y <= 0 || (long) size.X * size.Y > _limits.MaximumPixels)
                throw new ArgumentOutOfRangeException(nameof(size), "Spatial target exceeds its pixel budget.");
            if (_output != null && _output.Size == size) return;
            var state = new SpatialGlState();
            var created = new List<RenderTexture>();
            try
            {
                GL.ActiveTexture(TextureUnit.Texture0);
                var opaque = Target(RenderTargetColorFormat.Rgba8Srgb, true, "opaque");
                var accumulation = Target(RenderTargetColorFormat.Rgba32F, false, "accumulation");
                var revealage = Target(RenderTargetColorFormat.R32F, false, "revealage");
                var output = Target(RenderTargetColorFormat.Rgba8Srgb, false, "output");
                var depth = _owner.RtToLoaded(opaque).DepthStencilHandle;
                foreach (var target in new[] { accumulation, revealage })
                {
                    GL.BindFramebuffer(FramebufferTarget.DrawFramebuffer, _owner.RtToLoaded(target).FramebufferHandle.Handle);
                    GL.FramebufferRenderbuffer(FramebufferTarget.DrawFramebuffer, FramebufferAttachment.DepthAttachment,
                        RenderbufferTarget.Renderbuffer, depth.Handle);
                    if (GL.CheckFramebufferStatus(FramebufferTarget.DrawFramebuffer) != FramebufferErrorCode.FramebufferComplete)
                        throw new InvalidOperationException("Spatial transparency target is incomplete.");
                }
                _opaque?.Dispose(); _accumulation?.Dispose(); _revealage?.Dispose(); _output?.Dispose();
                (_opaque, _accumulation, _revealage, _output) = (opaque, accumulation, revealage, output);
                created.Clear();
            }
            finally
            {
                foreach (var target in created) target.Dispose();
                state.Restore();
            }
            return;
            RenderTexture Target(RenderTargetColorFormat format, bool depth, string name)
            {
                var target = (RenderTexture) _owner.CreateRenderTarget(size, new RenderTargetFormatParameters(format, depth), name: $"spatial-{name}");
                created.Add(target);
                return target;
            }
        }

        public SpatialRenderResult Render(PerspectiveCamera camera, SpatialRenderScene scene)
        {
            CheckThread(); ArgumentNullException.ThrowIfNull(scene);
            if (!camera.Pose.IsValid || camera.AspectRatio <= 0) throw new ArgumentException("Invalid spatial camera.");
            _visible.Clear(); _required.Clear(); _requiredKeys.Clear(); _sceneKeys.Clear();
            SelectShadows(camera, scene);
            long required = 0;
            foreach (var instance in scene.Instances)
            {
                var key = (instance.Mesh.Source.OwnerFrame, instance.Mesh.Key);
                _sceneKeys.Add(key);
                var visible = camera.Frustum.Intersects(instance.Mesh.Bounds, instance.FramePose);
                if (visible) _visible.Add(instance);
                var shadowCaster = _shadowViews.Any(view => view.Frustum.Intersects(instance.Mesh.Bounds, instance.FramePose))
                    && instance.Mesh.Faces.Any(face => scene.Materials[face.MaterialProfile].AlphaMode != SpatialAlphaMode.Transparent);
                if (!visible && !shadowCaster) continue;
                _required.Add(instance); _requiredKeys.Add(key);
                required = checked(required + instance.Mesh.EstimatedBufferBytes);
            }
            // Finish queued 2D/UI operations before borrowing the GL context.
            _owner.FlushRenderQueue();
            var state = new SpatialGlState();
            var uploaded = 0;
            var draws = 0;
            try
            {
                PrepareState();
                GL.ActiveTexture(TextureUnit.Texture0);
                foreach (var key in _meshes.Keys.Where(k => !_sceneKeys.Contains(k)).ToArray()) Remove(key);
                if (required > _limits.MaximumMeshBytes)
                {
                    ClearOutput(scene.ClearColor);
                    return Result(SpatialRenderStatus.BudgetExceeded);
                }
                // Retire every changed buffer before allocating replacements. A
                // growing chunk can otherwise exhaust the budget while a later
                // shrinking chunk still holds its old allocation.
                foreach (var instance in _required)
                {
                    var mesh = instance.Mesh;
                    var key = (mesh.Source.OwnerFrame, mesh.Key);
                    if (_meshes.TryGetValue(key, out var gpu) && !ReferenceEquals(gpu.Mesh, mesh))
                    {
                        if (gpu.Mesh.VertexMemory.Equals(mesh.VertexMemory) && gpu.Mesh.IndexMemory.Equals(mesh.IndexMemory)
                            || gpu.Mesh.VertexMemory.Span.SequenceEqual(mesh.VertexMemory.Span)
                            && gpu.Mesh.IndexMemory.Span.SequenceEqual(mesh.IndexMemory.Span)) gpu.Mesh = mesh;
                        else Remove(key);
                    }
                }
                var ready = true;
                foreach (var instance in _required)
                {
                    var mesh = instance.Mesh;
                    var key = (mesh.Source.OwnerFrame, mesh.Key);
                    if (!_meshes.TryGetValue(key, out var gpu))
                    {
                        while (_residentBytes + mesh.EstimatedBufferBytes > _limits.MaximumMeshBytes)
                            Remove(_meshes.Keys.First(k => !_requiredKeys.Contains(k)));
                        gpu = new SpatialGpuMesh(mesh);
                        _meshes.Add(key, gpu); _residentBytes += mesh.EstimatedBufferBytes;
                    }
                    uploaded += gpu.Upload(_limits.UploadBytesPerFrame - uploaded);
                    ready &= gpu.Ready;
                }
                if (!ready)
                {
                    // Never display an old wall or a scene with missing mandatory
                    // chunks while the replacement upload is incomplete.
                    ClearOutput(scene.ClearColor);
                    return Result(SpatialRenderStatus.Uploading);
                }
                draws += DrawShadows(scene);
                Bind(_opaque, true);
                GL.DepthMask(true); GL.ClearDepth(1);
                GL.ClearColor(scene.ClearColor.X, scene.ClearColor.Y, scene.ClearColor.Z, scene.ClearColor.W);
                GL.Clear(ClearBufferMask.ColorBufferBit | ClearBufferMask.DepthBufferBit);
                GL.Enable(EnableCap.DepthTest); GL.Disable(EnableCap.Blend);
                DrawPass(0);
                GL.DepthMask(false); GL.Enable(EnableCap.Blend);
                GL.BlendEquation(BlendEquationMode.FuncAdd);
                Bind(_accumulation, false); GL.ClearColor(0, 0, 0, 0); GL.Clear(ClearBufferMask.ColorBufferBit);
                GL.BlendFunc(BlendingFactor.One, BlendingFactor.One); DrawPass(1);
                Bind(_revealage, false); GL.ClearColor(1, 1, 1, 1); GL.Clear(ClearBufferMask.ColorBufferBit);
                GL.BlendFunc(BlendingFactor.Zero, BlendingFactor.OneMinusSrcAlpha); DrawPass(2);
                Bind(_output, true); GL.Disable(EnableCap.DepthTest); GL.Disable(EnableCap.Blend); GL.Disable(EnableCap.CullFace);
                GL.UseProgram(_composite.Handle); GL.BindVertexArray(_fullscreenVao);
                BindTexture(0, _opaque.Texture); BindTexture(1, _accumulation.Texture); BindTexture(2, _revealage.Texture);
                _composite.SetUniform("uOpaque", 0); _composite.SetUniform("uAccumulation", 1); _composite.SetUniform("uRevealage", 2);
                GL.DrawArrays(PrimitiveType.Triangles, 0, 3); draws++;
                _owner.FenceRenderTarget(_output); _owner.CheckGlError();
                return Result(SpatialRenderStatus.Ready);
            }
            finally { state.Restore(); _owner.CheckGlError(); }

            SpatialRenderResult Result(SpatialRenderStatus status) => new(status, _visible.Count, draws, _residentBytes, uploaded, _shadowViews.Count);
            void DrawPass(int pass)
            {
                GL.UseProgram(_forward.Handle);
                _forward.SetUniform("uViewProjection", camera.ViewProjection, transpose: false);
                _forward.SetUniform("uAmbient", scene.Ambient); _forward.SetUniform("uTexture", 0); _forward.SetUniform("uPass", pass);
                SetShadowUniforms();
                foreach (var instance in _visible)
                {
                    var mesh = instance.Mesh;
                    var gpu = _meshes[(mesh.Source.OwnerFrame, mesh.Key)];
                    GL.BindVertexArray(gpu.Vao);
                    _forward.SetUniform("uModel", instance.FramePose.ToMatrix(), transpose: false);
                    SetLights(scene, instance);
                    for (var i = 0; i < mesh.Faces.Count;)
                    {
                        var first = mesh.Faces[i];
                        var end = i + 1;
                        while (end < mesh.Faces.Count && mesh.Faces[end].MaterialProfile == first.MaterialProfile) end++;
                        var material = scene.Materials[first.MaterialProfile];
                        if ((material.AlphaMode == SpatialAlphaMode.Transparent) == (pass != 0))
                        {
                            if (material.DoubleSided) GL.Disable(EnableCap.CullFace); else GL.Enable(EnableCap.CullFace);
                            SetMaterialTexture(_forward, material);
                            _forward.SetUniform("uEmission", material.Emission);
                            GL.DrawElements(PrimitiveType.Triangles, (end - i) * 6, DrawElementsType.UnsignedInt, (IntPtr) (first.FirstIndex * sizeof(int)));
                            draws++;
                        }
                        i = end;
                    }
                }
            }
        }

        private void SetLights(SpatialRenderScene scene, SpatialRenderInstance instance)
        {
            Span<SpatialRenderLight> selected = stackalloc SpatialRenderLight[16];
            Span<float> scores = stackalloc float[16];
            var count = 0;
            var center = instance.FramePose.TransformPoint(instance.Mesh.Bounds.Center);
            var extent = instance.Mesh.Bounds.Size.Length() * 0.5f;
            foreach (var light in scene.Lights)
            {
                var distance = Vector3.DistanceSquared(center, light.Position);
                if (distance > (light.Radius + extent) * (light.Radius + extent)) continue;
                var score = distance / (light.Radius * light.Radius);
                var at = count;
                while (at > 0 && scores[at - 1] > score) at--;
                if (at >= 16) continue;
                count = Math.Min(count + 1, 16);
                for (var i = count - 1; i > at; i--) { selected[i] = selected[i - 1]; scores[i] = scores[i - 1]; }
                selected[at] = light; scores[at] = score;
            }
            _forward.SetUniform("uLightCount", count);
            for (var i = 0; i < count; i++)
            {
                var light = selected[i];
                _forward.SetUniform($"uLightPositionRadius[{i}]", new Vector4(light.Position, light.Radius));
                _forward.SetUniform($"uLightColorKind[{i}]", new Vector4(light.Color, (float) light.Kind));
                _forward.SetUniform($"uLightDirectionOuter[{i}]", new Vector4(light.Direction, light.OuterConeCosine));
                _forward.SetUniform($"uLightInner[{i}]", light.InnerConeCosine);
                _forward.SetUniform($"uShadowBase[{i}]", _shadowBases.GetValueOrDefault(light, -1));
            }
        }

        private void Bind(RenderTexture target, bool srgb)
        {
            GL.BindFramebuffer(FramebufferTarget.DrawFramebuffer, _owner.RtToLoaded(target).FramebufferHandle.Handle);
            GL.Viewport(0, 0, target.Size.X, target.Size.Y);
            if (srgb) GL.Enable(EnableCap.FramebufferSrgb); else GL.Disable(EnableCap.FramebufferSrgb);
        }
        private void BindTexture(int unit, ClydeTexture texture)
        {
            GL.ActiveTexture(TextureUnit.Texture0 + unit);
            GL.BindTexture(TextureTarget.Texture2D, _owner._loadedTextures[texture.TextureId].OpenGLObject.Handle);
            GL.BindSampler(unit, _sampler);
        }
        private void ClearOutput(Vector4 color)
        {
            Bind(_output, true); GL.ClearColor(color.X, color.Y, color.Z, color.W); GL.Clear(ClearBufferMask.ColorBufferBit);
            _owner.FenceRenderTarget(_output);
        }
        private static void PrepareState()
        {
            GL.Disable(EnableCap.StencilTest); GL.Disable(EnableCap.ScissorTest); GL.Disable(EnableCap.PrimitiveRestart);
            GL.Disable(EnableCap.PolygonOffsetFill); GL.Disable(EnableCap.SampleAlphaToCoverage);
            GL.Disable(EnableCap.SampleCoverage); GL.Disable(EnableCap.RasterizerDiscard);
            GL.ColorMask(true, true, true, true); GL.DepthRange(0, 1); GL.DepthFunc(DepthFunction.Lequal);
            GL.CullFace(CullFaceMode.Back); GL.FrontFace(FrontFaceDirection.Ccw); GL.PolygonMode(MaterialFace.FrontAndBack, PolygonMode.Fill);
        }
        private void Remove((EntityUid, SpatialMeshChunkKey) key)
        {
            var gpu = _meshes[key]; _residentBytes -= gpu.Mesh.EstimatedBufferBytes; gpu.Dispose(); _meshes.Remove(key);
        }
        private void CheckThread()
        {
            if (_disposed) throw new ObjectDisposedException(nameof(SpatialRenderer));
            if (!_owner.IsMainThread()) throw new InvalidOperationException("Spatial rendering requires the Clyde thread.");
        }
        public void Dispose()
        {
            if (_disposed) return;
            CheckThread(); _disposed = true;
            foreach (var gpu in _meshes.Values) gpu.Dispose(); _meshes.Clear(); _residentBytes = 0;
            _opaque?.Dispose(); _accumulation?.Dispose(); _revealage?.Dispose(); _output?.Dispose();
            _forward?.Delete(); _composite?.Delete();
            _shadowProgram?.Delete(); _shadowTarget?.Dispose();
            if (_fullscreenVao != 0) GL.DeleteVertexArray(_fullscreenVao);
            if (_sampler != 0) GL.DeleteSampler(_sampler);
            if (_mipSampler != 0) GL.DeleteSampler(_mipSampler);
            _owner._spatialRenderers.Remove(this);
        }
    }

    private sealed class SpatialGpuMesh : IDisposable
    {
        public SpatialChunkMesh Mesh;
        public readonly int Vao;
        private readonly int _vertices, _indices, _vertexBytes;
        private int _uploaded;
        public bool Ready => _uploaded == Mesh.EstimatedBufferBytes;
        public SpatialGpuMesh(SpatialChunkMesh mesh)
        {
            Mesh = mesh; _vertexBytes = checked(mesh.Vertices.Count * 32);
            Vao = GL.GenVertexArray(); GL.BindVertexArray(Vao);
            _vertices = GL.GenBuffer(); _indices = GL.GenBuffer();
            try
            {
                GL.BindBuffer(BufferTarget.ArrayBuffer, _vertices);
                GL.BufferData(BufferTarget.ArrayBuffer, (IntPtr) _vertexBytes, IntPtr.Zero, BufferUsageHint.StaticDraw);
                GL.BindBuffer(BufferTarget.ElementArrayBuffer, _indices);
                GL.BufferData(BufferTarget.ElementArrayBuffer, (IntPtr) (mesh.Indices.Count * sizeof(int)), IntPtr.Zero, BufferUsageHint.StaticDraw);
                for (var i = 0; i < 3; i++) GL.EnableVertexAttribArray(i);
                GL.VertexAttribPointer(0, 3, VertexAttribPointerType.Float, false, 32, 0);
                GL.VertexAttribPointer(1, 3, VertexAttribPointerType.Float, false, 32, 12);
                GL.VertexAttribPointer(2, 2, VertexAttribPointerType.Float, false, 32, 24);
                var error = GL.GetError();
                if (error != ErrorCode.NoError) throw new InvalidOperationException($"Spatial mesh allocation failed: {error}.");
            }
            catch { Dispose(); throw; }
        }
        public unsafe int Upload(int budget)
        {
            var start = _uploaded;
            GL.BindVertexArray(Vao);
            if (_uploaded < _vertexBytes)
                Copy(BufferTarget.ArrayBuffer, _vertices, MemoryMarshal.AsBytes(Mesh.VertexMemory.Span), 0);
            if (_uploaded >= _vertexBytes)
                Copy(BufferTarget.ElementArrayBuffer, _indices, MemoryMarshal.AsBytes(Mesh.IndexMemory.Span), _vertexBytes);
            return _uploaded - start;
            void Copy(BufferTarget target, int buffer, ReadOnlySpan<byte> source, int baseOffset)
            {
                var offset = _uploaded - baseOffset;
                var amount = Math.Min(budget, source.Length - offset);
                if (amount <= 0) return;
                GL.BindBuffer(target, buffer);
                fixed (byte* ptr = source.Slice(offset, amount))
                    GL.BufferSubData(target, (IntPtr) offset, amount, (IntPtr) ptr);
                _uploaded += amount; budget -= amount;
            }
        }
        public void Dispose() { GL.DeleteBuffer(_vertices); GL.DeleteBuffer(_indices); GL.DeleteVertexArray(Vao); }
    }
}
