in vec3 vWorld;
in vec3 vNormal;
in vec2 vUv;
uniform sampler2D uTexture;
uniform vec4 uUvRect;
uniform vec2 uRepeat;
uniform vec4 uColor;
uniform vec3 uEmission;
uniform vec3 uAmbient;
uniform float uCutoff;
uniform int uPass;
uniform int uMip;
uniform int uPremultiplied;
uniform int uLightCount;
uniform vec4 uLightPositionRadius[16];
uniform vec4 uLightColorKind[16];
uniform vec4 uLightDirectionOuter[16];
uniform float uLightInner[16];
uniform int uShadowBase[16];
uniform sampler2D uShadowAtlas;
uniform mat4 uShadowMatrices[24];
uniform vec4 uShadowRects[24];
uniform vec2 uShadowTexel;
uniform float uShadowBias;
out vec4 fragColor;
float shadowVisibility(int baseFace, vec3 fromLight, float cosine) {
    if (baseFace < 0) return 1.0;
    vec3 a = abs(fromLight);
    int face;
    if (a.x >= a.y && a.x >= a.z) face = fromLight.x >= 0.0 ? 0 : 1;
    else if (a.y >= a.z) face = fromLight.y >= 0.0 ? 2 : 3;
    else face = fromLight.z >= 0.0 ? 4 : 5;
    int index = baseFace + face;
    vec4 clip = uShadowMatrices[index] * vec4(vWorld, 1.0);
    if (clip.w <= 0.0) return 1.0;
    vec3 projected = clip.xyz / clip.w * 0.5 + 0.5;
    if (projected.z <= 0.0 || projected.z >= 1.0) return 1.0;
    vec4 rect = uShadowRects[index];
    vec2 uv = rect.xy + projected.xy * rect.zw;
    float depth = projected.z - uShadowBias * (1.0 + 3.0 * (1.0 - max(0.0, cosine)));
    float visibility = 0.0;
    // Four nearest depth comparisons, clamped within this face's atlas tile.
    for (int y = 0; y < 2; y++) for (int x = 0; x < 2; x++) {
        vec2 sampleUv = clamp(uv + (vec2(x, y) - 0.5) * uShadowTexel,
            rect.xy + uShadowTexel * 0.5, rect.xy + rect.zw - uShadowTexel * 0.5);
        visibility += depth <= texture(uShadowAtlas, sampleUv).r ? 0.25 : 0.0;
    }
    return visibility;
}
void main() {
    vec2 repeated = vUv * uRepeat;
    vec2 uv = uUvRect.xy + fract(repeated) * uUvRect.zw;
    // Derivatives come from the unwrapped coordinates: fract seams must not
    // suddenly select the coarsest mip level.
    vec4 sampled = uMip != 0 ? textureGrad(uTexture, uv, dFdx(repeated), dFdy(repeated)) : texture(uTexture, uv);
    if (uPremultiplied != 0) sampled.rgb = sampled.a > 0.0 ? sampled.rgb / sampled.a : vec3(0.0);
    vec4 albedo = sampled * uColor;
    if (albedo.a < uCutoff) discard;
    vec3 n = normalize(vNormal) * (gl_FrontFacing ? 1.0 : -1.0);
    vec3 light = uAmbient;
    for (int i = 0; i < 16; i++) {
        if (i >= uLightCount) break;
        vec3 delta = uLightPositionRadius[i].xyz - vWorld;
        float distanceSquared = dot(delta, delta);
        float radius = uLightPositionRadius[i].w;
        float falloff = max(0.0, 1.0 - distanceSquared / (radius * radius));
        vec3 direction = delta * inversesqrt(max(distanceSquared, 0.000001));
        float cone = 1.0;
        if (uLightColorKind[i].w > 0.5)
            cone = smoothstep(uLightDirectionOuter[i].w, uLightInner[i], dot(-direction, uLightDirectionOuter[i].xyz));
        float cosine = dot(n, direction);
        light += uLightColorKind[i].rgb * max(0.0, cosine) * falloff * falloff * cone
            * shadowVisibility(uShadowBase[i], -delta, cosine);
    }
    vec3 color = albedo.rgb * light + uEmission;
    if (uPass == 1) {
        float weight = clamp(pow(min(1.0, albedo.a * 10.0) + 0.01, 3.0) * 1e8
            * pow(1.0 - gl_FragCoord.z * 0.9, 3.0), 1e-2, 3e3);
        fragColor = vec4(color * albedo.a, albedo.a) * weight;
    } else if (uPass == 2) {
        fragColor = vec4(albedo.a);
    } else {
        fragColor = vec4(color, 1.0);
    }
}
