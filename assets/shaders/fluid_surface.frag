#version 410 core
layout(location = 0) in  vec3 v_Position;
layout(location = 1) in  vec3 v_Normal;
layout(location = 0) out vec4 f_Color;

uniform vec3  u_Color;
uniform float u_Alpha;
uniform vec3  u_LightDir;
uniform vec3  u_LightColor;
uniform vec3  u_AmbientColor;
uniform vec3  u_ViewPosition;
uniform float u_Shininess;

void main() {
    vec3 N = normalize(v_Normal);
    if (! gl_FrontFacing)
        N = -N;

    vec3 L = normalize(u_LightDir);
    vec3 V = normalize(u_ViewPosition - v_Position);
    vec3 H = normalize(L + V);

    float fresnel = pow(1.0 - max(dot(N, V), 0.0), 3.0);
    float diff = max(dot(N, L), 0.0);
    float spec = pow(max(dot(N, H), 0.0), u_Shininess);

    vec3 color = u_AmbientColor * u_Color
        + diff * u_LightColor * u_Color
        + spec * u_LightColor * 0.45
        + fresnel * vec3(0.35, 0.55, 0.75);

    f_Color = vec4(color, clamp(u_Alpha, 0.0, 1.0));
}