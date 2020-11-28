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

struct Material
{
    vec3 ambient;
    vec3 diffuse;
    vec3 specular;
    float shininess;
};
uniform Material material;

struct Light
{
    vec3 position;
    vec3 ambient;
    vec3 diffuse;
    vec3 specular;
};
uniform Light light;

void main()
{
    // ambient
    vec3 ambient = light.ambient * material.ambient;

    // diff
    vec3 norm = normalize(normalFrag);
    vec3 lightDir = normalize(light.position - posFrag);
    float diff = max(dot(norm, lightDir), 0.0);
    vec3 diffuse = light.diffuse * diff * material.diffuse;

    // specular
    vec3 viewDir = normalize(viewPos - posFrag);
    vec3 reflectDir = reflect(-lightDir, norm);
    float spec = pow(max(dot(viewDir, reflectDir), 0.0), material.shininess);
    vec3 specular = light.specular * spec * material.specular;

    // result
    vec3 result = (ambient + diffuse + specular) * colorFrag;
    colorFragOut = vec4(result, 1.0f);
}