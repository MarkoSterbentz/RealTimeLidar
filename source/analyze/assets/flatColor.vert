#version 130

in vec3 vertex;
in vec3 normal;
uniform mat4 mvMat;
uniform mat4 mvpMat;
out float shine;

void main() {
//    normal = normalize(mvpMat * vec4(vertex, 0.0)).xyz;

    vec3 transformedNormal = (mvMat * vec4(normal, 0.0)).xyz;
    shine = pow(max(0.0, dot(transformedNormal, vec3(0.0, 0.0, 1.0))), 2.0);

//    shine = max(0.0, dot(normal, vec3(0.0, 0.0, 1.0)));

    gl_Position = mvpMat * vec4(vertex, 1.0);
}