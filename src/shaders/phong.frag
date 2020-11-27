#version 450 core

in vec3 colorFrag;
in vec2 texFrag;
in vec3 normalFrag;
in vec3 posFrag;

out vec4 colorFragOut;

uniform vec3 viewPos;
uniform vec3 lightPos;
uniform vec3 lightColor;
uniform sampler2D textureSampler;

void main()
{
    // ambient
    float ambient = 0.1f;

    // diff
    vec3 norm = normalize(normalFrag);
    vec3 lightDir = normalize(lightPos - posFrag);
    float diff = max(dot(norm, lightDir), 0.0);

    // specular
    float specularStrength = 0.5f;
    vec3 viewDir = normalize(viewPos - posFrag);
    vec3 reflectDir = reflect(-lightDir, norm);
    float spec = pow(max(dot(viewDir, reflectDir), 0.0), 32);
    float specular = specularStrength * spec;

    // result
    vec3 result = (ambient + diff + specular) * lightColor * colorFrag;
    colorFragOut = vec4(result, 1.0f);
}