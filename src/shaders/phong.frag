#version 450 core

in vec3 colorFrag;
in vec2 texFrag;
in vec3 normalFrag;
in vec3 posFrag;

out vec4 colorFragOut;

uniform vec3 lightPos;
uniform vec3 lightColor;
uniform sampler2D textureSampler;

void main()
{
    float ambient = 0.1f;

    vec3 norm = normalize(normalFrag);
    vec3 lightDir = normalize(lightPos - posFrag);
    float diff = max(dot(norm, lightDir), 0.0);

    vec3 result = (ambient + diff) * lightColor * colorFrag;
    colorFragOut = vec4(result, 1.0f);
}