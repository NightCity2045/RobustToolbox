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
uniform int uLightCount;
uniform vec4 uLightPositionRadius[16];
uniform vec4 uLightColorKind[16];
uniform vec4 uLightDirectionOuter[16];
uniform float uLightInner[16];
out vec4 fragColor;
void main() {
    vec2 uv = uUvRect.xy + fract(vUv * uRepeat) * uUvRect.zw;
    vec4 albedo = texture(uTexture, uv) * uColor;
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
        light += uLightColorKind[i].rgb * max(0.0, dot(n, direction)) * falloff * falloff * cone;
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
