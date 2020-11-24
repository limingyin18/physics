#version 450 core

in vec4 colorFragIn;
in vec2 texFragIn;
out vec4 colorFragOut;

uniform sampler2D textureSampler;

void main()
{
   colorFragOut = colorFragIn + texture(textureSampler, texFragIn);
}