using System;
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
        private void SetMaterialTexture(GLShaderProgram program, SpatialRenderMaterial material)
        {
            var texture = RenderHandle.ExtractTexture(material.Texture ?? _owner.GetStockTexture(ClydeStockTexture.White), null, out var region);
            var uv = RenderHandle.WorldTextureBoundsToUV(texture, region);
            var mip = material.Sampling == SpatialTextureSampling.PixelMip;
            if (mip)
            {
                var loaded = _owner._loadedTextures[texture.TextureId];
                if (loaded.SpatialMipLevels == 0 || region != new UIBox2(0, 0, texture.Width, texture.Height))
                    throw new ArgumentException("PixelMip sampling requires a complete isolated spatial material pyramid.");
                if (material.AlphaMode == SpatialAlphaMode.Cutout && loaded.SpatialMipAlphaCutoff != material.AlphaCutoff
                    || material.AlphaMode == SpatialAlphaMode.Transparent && loaded.SpatialMipAlphaCutoff != null)
                    throw new ArgumentException("Material alpha mode/threshold does not match its mip coverage profile.");
                program.SetUniform("uUvRect", new Vector4(0, 0, 1, 1));
            }
            else
            {
                // Existing atlases keep nearest sampling with a half-texel inset.
                // Their neighbors must never be used to generate shared mip levels.
                var inset = new Vector2(0.5f / texture.Width, 0.5f / texture.Height);
                program.SetUniform("uUvRect", new Vector4(uv.Left + inset.X, uv.Bottom + inset.Y,
                    Math.Max(0, uv.Width - inset.X * 2), Math.Max(0, uv.Height - inset.Y * 2)));
            }
            BindTexture(0, texture);
            GL.BindSampler(0, mip ? _mipSampler : _sampler);
            program.SetUniform("uMip", mip ? 1 : 0);
            program.SetUniform("uRepeat", material.Repeat); program.SetUniform("uColor", material.Color);
            program.SetUniform("uCutoff", material.AlphaMode == SpatialAlphaMode.Cutout ? material.AlphaCutoff : 0);
        }
    }
}
