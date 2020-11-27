#version 450 core

layout (location = 0) in vec3 position;
layout (location = 1) in vec3 normal;
layout (location = 2) in vec2 tex;
layout (location = 3) in vec3 colorVert;

uniform mat4 projection;
uniform mat4 view;
uniform mat4 model;


out vec3 colorFrag;
out vec2 texFrag;
out vec3 normalFrag;
out vec3 posFrag;

void main()
{
    gl_Position =  projection * view * model * vec4(position, 1.0);
    posFrag = vec3(model * vec4(position, 1.0));
    colorFrag = colorVert;
    texFrag = tex;
    normalFrag = mat3(transpose(inverse(model))) * normal;
}