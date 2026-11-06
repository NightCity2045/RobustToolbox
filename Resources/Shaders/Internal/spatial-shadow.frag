in vec2 vUv;
uniform sampler2D uTexture;
uniform vec4 uUvRect;
uniform vec2 uRepeat;
uniform vec4 uColor;
uniform float uCutoff;
uniform int uMip;
out float fragDepth;
void main() {
    vec2 repeated = vUv * uRepeat;
    vec2 uv = uUvRect.xy + fract(repeated) * uUvRect.zw;
    float alpha = uMip != 0 ? textureGrad(uTexture, uv, dFdx(repeated), dFdy(repeated)).a : texture(uTexture, uv).a;
    if (alpha * uColor.a < uCutoff) discard;
    fragDepth = gl_FragCoord.z;
}
