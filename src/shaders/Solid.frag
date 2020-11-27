#version 450 core

in vec3 colorFragIn;
in vec2 texFragIn;
out vec4 colorFragOut;

uniform sampler2D textureSampler;

void main()
{
   colorFragOut = vec4(colorFragIn, 1.0f) * texture(textureSampler, texFragIn);
}