using System;
using System.Collections.Generic;
using OpenToolkit.Graphics.OpenGL4;
using Robust.Client.Graphics.Spatial;
using Robust.Shared.Graphics;
using Robust.Shared.Maths;
using SixLabors.ImageSharp.PixelFormats;

namespace Robust.Client.Graphics.Clyde;

internal sealed partial class Clyde
{
    private readonly List<SpatialTextureUpload> _spatialTextureUploads = new();

    public ISpatialTextureUpload CreateSpatialTextureUpload(SpatialTextureMipChain chain, long maximumBytes, string? name = null)
    {
        ArgumentNullException.ThrowIfNull(chain);
        if (!SupportsSpatialRendering) throw new NotSupportedException("Spatial material upload requires the spatial GL profile.");
        if (!IsMainThread()) throw new InvalidOperationException("Spatial textures require the Clyde thread.");
        if (maximumBytes <= 0 || chain.Bytes > maximumBytes) throw new ArgumentOutOfRangeException(nameof(maximumBytes));
        var maximumSize = GL.GetInteger(GetPName.MaxTextureSize);
        if (chain.Levels[0].Width > maximumSize || chain.Levels[0].Height > maximumSize)
            throw new ArgumentException("Material texture exceeds the GPU dimension limit.");
        FlushRenderQueue();
        var upload = new SpatialTextureUpload(this, chain, name);
        _spatialTextureUploads.Add(upload);
        return upload;
    }

    private sealed class SpatialTextureUpload : ISpatialTextureUpload
    {
        private readonly Clyde _owner;
        private readonly ClydeTexture _texture = default!;
        private SpatialTextureMipChain? _chain;
        private int _level, _pixel;
        private bool _disposed;
        public long AllocatedBytes { get; }
        public long UploadedBytes { get; private set; }
        public bool IsReady => !_disposed && UploadedBytes == AllocatedBytes;
        public Texture Texture
        {
            get
            {
                if (_disposed) throw new ObjectDisposedException(nameof(SpatialTextureUpload));
                if (!IsReady) throw new InvalidOperationException("Material texture upload is incomplete.");
                return _texture;
            }
        }

        public SpatialTextureUpload(Clyde owner, SpatialTextureMipChain chain, string? name)
        {
            _owner = owner; _chain = chain; AllocatedBytes = chain.Bytes;
            var state = new SpatialGlState();
            using var unpack = new SpatialUnpackState();
            try
            {
                GL.ActiveTexture(TextureUnit.Texture0);
                var first = chain.Levels[0];
                _texture = owner.CreateBaseTextureInternal<Rgba32>(first.Width, first.Height,
                    new TextureLoadParameters { Srgb = chain.Srgb, SampleParameters = new TextureSampleParameters { Filter = false, WrapMode = Robust.Shared.Graphics.TextureWrapMode.Repeat } }, name);
                for (var level = 0; level < chain.Levels.Count; level++)
                {
                    var image = chain.Levels[level];
                    GL.TexImage2D(TextureTarget.Texture2D, level, chain.Srgb ? PixelInternalFormat.Srgb8Alpha8 : PixelInternalFormat.Rgba8,
                        image.Width, image.Height, 0, PixelFormat.Rgba, PixelType.UnsignedByte, IntPtr.Zero);
                }
                GL.TexParameter(TextureTarget.Texture2D, TextureParameterName.TextureMaxLevel, chain.Levels.Count - 1);
                var loaded = owner._loadedTextures[_texture.TextureId];
                loaded.SpatialMipLevels = chain.Levels.Count; loaded.SpatialMipAlphaCutoff = chain.AlphaCutoff;
                loaded.MemoryPressure = chain.Bytes;
                var error = GL.GetError();
                if (error != ErrorCode.NoError) throw new InvalidOperationException($"Material pyramid allocation failed: {error}.");
            }
            catch { _texture?.Dispose(); throw; }
            finally { state.Restore(); }
        }

        public unsafe int Upload(int maximumBytes)
        {
            CheckThread();
            if (maximumBytes < 4) throw new ArgumentOutOfRangeException(nameof(maximumBytes));
            if (IsReady) return 0;
            var chain = _chain!;
            _owner.FlushRenderQueue();
            var state = new SpatialGlState();
            using var unpack = new SpatialUnpackState();
            var uploaded = 0;
            try
            {
                GL.ActiveTexture(TextureUnit.Texture0);
                GL.BindTexture(TextureTarget.Texture2D, _owner._loadedTextures[_texture.TextureId].OpenGLObject.Handle);
                while (_level < chain.Levels.Count && maximumBytes - uploaded >= 4)
                {
                    var level = chain.Levels[_level];
                    var y = _pixel / level.Width; var x = _pixel % level.Width;
                    var pixels = Math.Min(level.Width - x, (maximumBytes - uploaded) / 4);
                    // GL rows run bottom-up; the immutable CPU pyramid stays in
                    // ordinary image coordinates. A budget may split a scanline.
                    var data = level.Pixels.Span.Slice((level.Height - 1 - y) * level.Width + x, pixels);
                    fixed (Rgba32* pointer = data)
                        GL.TexSubImage2D(TextureTarget.Texture2D, _level, x, y, pixels, 1, PixelFormat.Rgba, PixelType.UnsignedByte, (IntPtr) pointer);
                    var error = GL.GetError();
                    if (error != ErrorCode.NoError) throw new InvalidOperationException($"Material mip upload failed: {error}.");
                    _pixel += pixels; uploaded += pixels * 4; UploadedBytes += pixels * 4;
                    if (_pixel == level.Width * level.Height) { _level++; _pixel = 0; }
                }
                if (IsReady) _chain = null;
                return uploaded;
            }
            finally { state.Restore(); }
        }

        private void CheckThread()
        {
            if (_disposed) throw new ObjectDisposedException(nameof(SpatialTextureUpload));
            if (!_owner.IsMainThread()) throw new InvalidOperationException("Spatial textures require the Clyde thread.");
        }
        public void Dispose()
        {
            if (_disposed) return;
            CheckThread(); _disposed = true; _texture.Dispose(); _chain = null;
            _owner._spatialTextureUploads.Remove(this);
        }
    }

    private sealed class SpatialUnpackState : IDisposable
    {
        private readonly int _buffer = GL.GetInteger(GetPName.PixelUnpackBufferBinding);
        private readonly int _alignment = GL.GetInteger(GetPName.UnpackAlignment);
        private readonly int _rowLength = GL.GetInteger(GetPName.UnpackRowLength);
        private readonly int _skipRows = GL.GetInteger(GetPName.UnpackSkipRows);
        private readonly int _skipPixels = GL.GetInteger(GetPName.UnpackSkipPixels);
        public SpatialUnpackState()
        {
            GL.BindBuffer(BufferTarget.PixelUnpackBuffer, 0);
            GL.PixelStore(PixelStoreParameter.UnpackAlignment, 4);
            GL.PixelStore(PixelStoreParameter.UnpackRowLength, 0);
            GL.PixelStore(PixelStoreParameter.UnpackSkipRows, 0);
            GL.PixelStore(PixelStoreParameter.UnpackSkipPixels, 0);
        }
        public void Dispose()
        {
            GL.BindBuffer(BufferTarget.PixelUnpackBuffer, _buffer);
            GL.PixelStore(PixelStoreParameter.UnpackAlignment, _alignment);
            GL.PixelStore(PixelStoreParameter.UnpackRowLength, _rowLength);
            GL.PixelStore(PixelStoreParameter.UnpackSkipRows, _skipRows);
            GL.PixelStore(PixelStoreParameter.UnpackSkipPixels, _skipPixels);
        }
    }
}
