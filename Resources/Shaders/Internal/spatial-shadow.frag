in vec2 vUv;
uniform sampler2D uTexture;
uniform vec4 uUvRect;
uniform vec2 uRepeat;
uniform float uAlpha;
uniform float uCutoff;
out float fragDepth;
void main() {
    vec2 uv = uUvRect.xy + fract(vUv * uRepeat) * uUvRect.zw;
    if (texture(uTexture, uv).a * uAlpha < uCutoff) discard;
    fragDepth = gl_FragCoord.z;
}
