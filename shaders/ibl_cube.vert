#version 450

layout(push_constant, std430) uniform PC {
    mat4 mvp;
    float roughness;
    float resolution;
    float _pad0;
    float _pad1;
} pc;

layout(location = 0) out vec3 vLocalPos;

// Unit cube (36 verts) for cubemap face capture.
const vec3 kCube[36] = vec3[](
    vec3(-1, -1, -1), vec3(-1,  1, -1), vec3( 1,  1, -1),
    vec3( 1,  1, -1), vec3( 1, -1, -1), vec3(-1, -1, -1),

    vec3(-1, -1,  1), vec3( 1, -1,  1), vec3( 1,  1,  1),
    vec3( 1,  1,  1), vec3(-1,  1,  1), vec3(-1, -1,  1),

    vec3(-1,  1, -1), vec3(-1,  1,  1), vec3( 1,  1,  1),
    vec3( 1,  1,  1), vec3( 1,  1, -1), vec3(-1,  1, -1),

    vec3(-1, -1, -1), vec3( 1, -1, -1), vec3( 1, -1,  1),
    vec3( 1, -1,  1), vec3(-1, -1,  1), vec3(-1, -1, -1),

    vec3( 1, -1, -1), vec3( 1,  1, -1), vec3( 1,  1,  1),
    vec3( 1,  1,  1), vec3( 1, -1,  1), vec3( 1, -1, -1),

    vec3(-1, -1, -1), vec3(-1, -1,  1), vec3(-1,  1,  1),
    vec3(-1,  1,  1), vec3(-1,  1, -1), vec3(-1, -1, -1)
);

void main() {
    vLocalPos = kCube[gl_VertexIndex];
    gl_Position = pc.mvp * vec4(vLocalPos, 1.0);
}
