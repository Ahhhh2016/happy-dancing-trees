#version 330 core
out vec4 fragColor;

// Additional information for lighting
in vec4 normal_worldSpace;
in vec4 position_worldSpace;

uniform int wire = 0;
uniform float red = 1.0;
uniform float green = 1.0;
uniform float blue = 1.0;
uniform float alpha = 1.0;

void main() {
    if (wire == 1) {
        fragColor = vec4(0.0, 0.0, 0.0, 1);
        return;
    }
    vec3 N = normalize(normal_worldSpace.xyz);
    vec3 L = normalize(vec3(-2.0, 2.0, -3.0) - position_worldSpace.xyz);
    vec3 V = normalize(vec3(0.0, 0.0, -5.0) - position_worldSpace.xyz);
    vec3 H = normalize(L + V);

    // Two-sided diffuse helps preserve a soft "inflated balloon" read.
    float diffuse = max(abs(dot(N, L)), 0.0);
    float spec = pow(max(dot(N, H), 0.0), 72.0);
    float rim = pow(1.0 - max(dot(N, V), 0.0), 2.5);

    vec3 baseColor = vec3(red, green, blue);
    vec3 color = baseColor * (0.22 + 0.78 * diffuse); // ambient + diffuse
    color += vec3(1.0) * (0.85 * spec);               // glossy white highlight
    color += vec3(1.0, 0.85, 0.95) * (0.18 * rim);    // subtle candy/plastic edge glow

    fragColor = vec4(clamp(color, 0.0, 1.0), alpha);
}
