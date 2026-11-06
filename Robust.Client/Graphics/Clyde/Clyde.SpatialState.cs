using OpenToolkit.Graphics.OpenGL4;

namespace Robust.Client.Graphics.Clyde;

internal sealed partial class Clyde
{
    /// <summary>Raw GL scope leaves Clyde's queued 2D state and cached bindings unchanged.</summary>
    private sealed class SpatialGlState
    {
        private static readonly EnableCap[] Capabilities =
        {
            EnableCap.DepthTest, EnableCap.Blend, EnableCap.CullFace, EnableCap.StencilTest, EnableCap.ScissorTest,
            EnableCap.FramebufferSrgb, EnableCap.PrimitiveRestart, EnableCap.PolygonOffsetFill,
            EnableCap.SampleAlphaToCoverage, EnableCap.SampleCoverage, EnableCap.RasterizerDiscard
        };
        private readonly bool[] _enabled = new bool[Capabilities.Length];
        private readonly int[] _viewport = new int[4];
        private readonly float[] _clearColor = new float[4];
        private readonly bool[] _colorMask = new bool[4];
        private readonly double[] _depthRange = new double[2];
        private readonly int[] _textures = new int[4];
        private readonly int[] _samplers = new int[4];
        private readonly int _draw, _read, _program, _vao, _arrayBuffer, _activeTexture, _renderbuffer;
        private readonly int _depthFunc, _cullFace, _frontFace, _blendSrcRgb, _blendDstRgb, _blendSrcAlpha, _blendDstAlpha, _equationRgb, _equationAlpha;
        private readonly bool _depthWrite;
        private readonly double _clearDepth;
        private readonly int[] _polygonMode = new int[2];

        public SpatialGlState()
        {
            _draw = GL.GetInteger(GetPName.DrawFramebufferBinding); _read = GL.GetInteger(GetPName.ReadFramebufferBinding);
            _program = GL.GetInteger(GetPName.CurrentProgram); _vao = GL.GetInteger(GetPName.VertexArrayBinding);
            _arrayBuffer = GL.GetInteger(GetPName.ArrayBufferBinding); _activeTexture = GL.GetInteger(GetPName.ActiveTexture);
            _renderbuffer = GL.GetInteger(GetPName.RenderbufferBinding);
            _depthFunc = GL.GetInteger(GetPName.DepthFunc); _depthWrite = GL.GetBoolean(GetPName.DepthWritemask);
            _clearDepth = GL.GetDouble(GetPName.DepthClearValue);
            _cullFace = GL.GetInteger(GetPName.CullFaceMode); _frontFace = GL.GetInteger(GetPName.FrontFace);
            _blendSrcRgb = GL.GetInteger(GetPName.BlendSrcRgb); _blendDstRgb = GL.GetInteger(GetPName.BlendDstRgb);
            _blendSrcAlpha = GL.GetInteger(GetPName.BlendSrcAlpha); _blendDstAlpha = GL.GetInteger(GetPName.BlendDstAlpha);
            _equationRgb = GL.GetInteger(GetPName.BlendEquationRgb); _equationAlpha = GL.GetInteger(GetPName.BlendEquationAlpha);
            GL.GetInteger(GetPName.Viewport, _viewport); GL.GetFloat(GetPName.ColorClearValue, _clearColor);
            GL.GetBoolean(GetPName.ColorWritemask, _colorMask); GL.GetDouble(GetPName.DepthRange, _depthRange);
            GL.GetInteger(GetPName.PolygonMode, _polygonMode);
            for (var i = 0; i < Capabilities.Length; i++) _enabled[i] = GL.IsEnabled(Capabilities[i]);
            for (var i = 0; i < 4; i++)
            {
                GL.ActiveTexture(TextureUnit.Texture0 + i);
                _textures[i] = GL.GetInteger(GetPName.TextureBinding2D);
                _samplers[i] = GL.GetInteger(GetPName.SamplerBinding);
            }
            GL.ActiveTexture((TextureUnit) _activeTexture);
        }

        public void Restore()
        {
            GL.BindFramebuffer(FramebufferTarget.DrawFramebuffer, _draw); GL.BindFramebuffer(FramebufferTarget.ReadFramebuffer, _read);
            GL.UseProgram(_program); GL.BindVertexArray(_vao); GL.BindBuffer(BufferTarget.ArrayBuffer, _arrayBuffer);
            GL.BindRenderbuffer(RenderbufferTarget.Renderbuffer, _renderbuffer);
            GL.Viewport(_viewport[0], _viewport[1], _viewport[2], _viewport[3]);
            GL.ClearColor(_clearColor[0], _clearColor[1], _clearColor[2], _clearColor[3]); GL.ClearDepth(_clearDepth);
            GL.DepthRange(_depthRange[0], _depthRange[1]); GL.DepthMask(_depthWrite); GL.DepthFunc((DepthFunction) _depthFunc);
            GL.ColorMask(_colorMask[0], _colorMask[1], _colorMask[2], _colorMask[3]);
            GL.CullFace((CullFaceMode) _cullFace); GL.FrontFace((FrontFaceDirection) _frontFace);
            if (_polygonMode[0] == _polygonMode[1])
                GL.PolygonMode(MaterialFace.FrontAndBack, (PolygonMode) _polygonMode[0]);
            else
            {
                // Separate face modes exist only in compatibility contexts.
                GL.PolygonMode(MaterialFace.Front, (PolygonMode) _polygonMode[0]);
                GL.PolygonMode(MaterialFace.Back, (PolygonMode) _polygonMode[1]);
            }
            GL.BlendFuncSeparate((BlendingFactorSrc) _blendSrcRgb, (BlendingFactorDest) _blendDstRgb,
                (BlendingFactorSrc) _blendSrcAlpha, (BlendingFactorDest) _blendDstAlpha);
            GL.BlendEquationSeparate((BlendEquationMode) _equationRgb, (BlendEquationMode) _equationAlpha);
            for (var i = 0; i < Capabilities.Length; i++)
                if (_enabled[i]) GL.Enable(Capabilities[i]); else GL.Disable(Capabilities[i]);
            for (var i = 0; i < 4; i++)
            {
                GL.ActiveTexture(TextureUnit.Texture0 + i);
                GL.BindTexture(TextureTarget.Texture2D, _textures[i]); GL.BindSampler(i, _samplers[i]);
            }
            GL.ActiveTexture((TextureUnit) _activeTexture);
        }
    }
}
