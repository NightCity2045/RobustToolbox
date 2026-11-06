in vec3 aPosition;
in vec2 aUv;
uniform mat4 uModel;
uniform mat4 uViewProjection;
out vec2 vUv;
void main() {
    vUv = aUv;
    gl_Position = uViewProjection * uModel * vec4(aPosition, 1.0);
}
