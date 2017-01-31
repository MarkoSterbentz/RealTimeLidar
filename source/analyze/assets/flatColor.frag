#version 130

uniform vec3 color;
in float shine;
out vec4 fragColor;

void main() {
//    float camSpecular = max(0.0, dot(normal, vec3(0.0, 0.0, -1.0)));
//    vec3 shine = vec3(0.5 * camSpecular, 0.5 * camSpecular, 0.5 * camSpecular);
//    fragColor = vec4(color + shine, 1.0);
    fragColor = vec4(color + vec3(shine * 0.4), 1.0);
}