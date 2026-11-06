in vec3 aPosition;
in vec3 aNormal;
in vec2 aUv;
uniform mat4 uModel;
uniform mat4 uViewProjection;
out vec3 vWorld;
out vec3 vNormal;
out vec2 vUv;
void main() {
    vec4 world = uModel * vec4(aPosition, 1.0);
    vWorld = world.xyz;
    vNormal = mat3(uModel) * aNormal;
    vUv = aUv;
    gl_Position = uViewProjection * world;
}
