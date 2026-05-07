#version 330 core
out vec4 fragColor;

// Additional information for lighting
in vec4 normal_worldSpace;
in vec4 position_worldSpace;
in vec2 texCoord;

uniform int wire = 0;
uniform float red = 1.0;
uniform float green = 1.0;
uniform float blue = 1.0;
uniform float alpha = 1.0;
uniform int useTexture = 0;
uniform sampler2D diffuseTex;

// Scene lighting (set from C++ each frame).
uniform mat4 view;
uniform vec3 ambientColor;
// Light position in view space (fixed relative to camera): orbit/rotate view
// does not swing the highlight across the mesh the way a world-space lamp does.
uniform vec3 keyLightPosView;

void main() {
    if (wire == 1) {
        fragColor = vec4(0.0, 0.0, 0.0, 1);
        return;
    }
    vec3 pView = (view * position_worldSpace).xyz;
    vec3 nView = normalize(mat3(view) * normalize(normal_worldSpace.xyz));
    vec3 L = normalize(keyLightPosView - pView);
    vec3 V = normalize(-pView);
    vec3 H = normalize(L + V);

    // Two-sided diffuse helps preserve a soft "inflated balloon" read.
    float diffuse = max(abs(dot(nView, L)), 0.0);
    float spec = pow(max(dot(nView, H), 0.0), 72.0);
    float rim = pow(1.0 - max(dot(nView, V), 0.0), 2.5);

    vec3 baseColor = vec3(red, green, blue);
    if (useTexture != 0) {
        // Canvas/OBJ vs OpenGL: flip both axes so sampling matches the 2D editor.
        baseColor = texture(diffuseTex, vec2(1.0 - texCoord.x, 1.0 - texCoord.y)).rgb;
    }
    // Ambient (floor brightness) + directional; diffuse scale lowered vs old 0.78
    // so a brighter ambient does not push most pixels into clamp(…,1.0).
    vec3 color = baseColor * ambientColor;
    color += baseColor * (0.48 * diffuse);
    color += vec3(1.0) * (0.85 * spec);               // glossy white highlight
    color += vec3(1.0, 0.85, 0.95) * (0.18 * rim);    // subtle candy/plastic edge glow

    fragColor = vec4(clamp(color, 0.0, 1.0), alpha);
}
