in vec2 vUv;
uniform sampler2D uOpaque;
uniform sampler2D uAccumulation;
uniform sampler2D uRevealage;
out vec4 fragColor;
void main() {
    vec4 opaque = texture(uOpaque, vUv);
    vec4 accumulation = texture(uAccumulation, vUv);
    float revealage = clamp(texture(uRevealage, vUv).r, 0.0, 1.0);
    vec3 transparent = accumulation.rgb / max(accumulation.a, 0.00001);
    fragColor = vec4(transparent * (1.0 - revealage) + opaque.rgb * revealage,
        1.0 - revealage + opaque.a * revealage);
}
