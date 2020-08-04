#version 450 core

layout (location = 0) in vec3 position;
layout (location = 1) in vec3 normal;
layout (location = 2) in vec2 tex;
layout (location = 3) in vec4 colorVert;

uniform mat4 projection;
uniform mat4 view;
uniform mat4 model;

out vec4 colorFragIn;
out vec2 texFragIn;

void main()
{
    gl_Position =  projection * view * model * vec4(position.x, position.y, position.z, 1.0);
    colorFragIn = colorVert;
    texFragIn = tex;
}