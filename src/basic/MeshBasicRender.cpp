#include "MeshBasicRender.hpp"

using namespace Eigen;

void MeshBasicRender::draw()
{
	bindShader();
	_draw();
	unBindShader();
}

void MeshBasicRender::_draw()
{
	glDrawElements(GL_TRIANGLES, (GLsizei)meshBase->indices.size(), GL_UNSIGNED_INT, 0);
}

void MeshBasicRender::setMesh(MeshBase* m)
{
	meshBase = m;
	setVAO();
	addShader();
	linkShader();
	addUniforms();
}

void MeshBasicRender::setTexImg(int w, int h, unsigned char* data)
{
	glGenTextures(1, &tex);
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, tex);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, w, h, 0, GL_RGB, GL_UNSIGNED_BYTE, data);
	glGenerateMipmap(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, 0);
}

void MeshBasicRender::setVAO()
{
 	// vertex array
	glGenVertexArrays(1, &vao);
	glBindVertexArray(vao);

	// vertex buffer
	glGenBuffers(1, &vbo);
	glBindBuffer(GL_ARRAY_BUFFER, vbo);
	glBufferData(GL_ARRAY_BUFFER, sizeof(MeshBase::Vertex) * meshBase->data.size(), meshBase->data.data(), GL_DYNAMIC_DRAW);

	// index buffer
	glGenBuffers(1, &ebo);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned) * meshBase->indices.size(), meshBase->indices.data(), GL_STATIC_DRAW);

	// position attribute
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(MeshBase::Vertex), (GLvoid*)0);
	glEnableVertexAttribArray(0);

	// normal attribute
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(MeshBase::Vertex), (GLvoid*)(3 * sizeof(float)));
	glEnableVertexAttribArray(1);

	// texture attribute
	glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(MeshBase::Vertex), (GLvoid *)(6 * sizeof(float)));
	glEnableVertexAttribArray(2);

	// color attribute
	glVertexAttribPointer(3, 3, GL_FLOAT, GL_FALSE, sizeof(MeshBase::Vertex), (GLvoid *)(8 * sizeof(float)));
	glEnableVertexAttribArray(3);
}

void MeshBasicRender::bindShader()
{
	glUseProgram(render.Program);
	glBindVertexArray(vao);
	glUniformMatrix4fv(render.uniforms.at("projection"), 1, GL_FALSE, camera->projection.data());
	glUniformMatrix4fv(render.uniforms.at("view"), 1, GL_FALSE, camera->view.data());
	glUniformMatrix4fv(render.uniforms.at("model"), 1, GL_FALSE, model.data());
}

void MeshBasicRender::unBindShader()
{
	glBindVertexArray(0);
	glUseProgram(0);
}

void MeshBasicRender::addShader()
{
    render.addShader("basic.vert", GL_VERTEX_SHADER);
    render.addShader("basic.frag", GL_FRAGMENT_SHADER);
}

void MeshBasicRender::linkShader()
{
	render.link();
}

void MeshBasicRender::addUniforms()
{
	render.addUniform("projection");
	render.addUniform("view");
	render.addUniform("model");
}

void SolidRender::addShader()
{
    render.addShader("Solid.vert", GL_VERTEX_SHADER);
    render.addShader("Solid.frag", GL_FRAGMENT_SHADER);
}

void SolidRender::addUniforms()
{
	MeshBasicRender::addUniforms();
	render.addUniform("textureSampler");
}

void SolidRender::bindShader()
{
	MeshBasicRender::bindShader();
	glBindTexture(GL_TEXTURE_2D, tex);
}

void SolidRender::unBindShader()
{
	glBindTexture(GL_TEXTURE_2D, 0);
	MeshBasicRender::unBindShader();
}

void CeramicRender::addShader()
{
    render.addShader("ceramic.vert", GL_VERTEX_SHADER);
    render.addShader("ceramic.frag", GL_FRAGMENT_SHADER);
}

void PhongRender::addShader()
{
	render.addShader("phong.vert", GL_VERTEX_SHADER);
    render.addShader("phong.frag", GL_FRAGMENT_SHADER);
}

void PhongRender::addUniforms()
{
	MeshBasicRender::addUniforms();
	render.addUniform("lightColor");
	render.addUniform("lightPos");
	render.addUniform("viewPos");
	render.addUniform("material.ambient");
	render.addUniform("material.diffuse");
	render.addUniform("material.specular");
	render.addUniform("material.shininess");
}

void PhongRender::bindShader()
{
	MeshBasicRender::bindShader();
	glUniform3fv(render.uniforms.at("lightColor"), 1, lightColor.data());
	glUniform3fv(render.uniforms.at("lightPos"), 1, lightPos.data());
	glUniform3fv(render.uniforms.at("viewPos"), 1, camera->getCameraPos().data());

	glUniform3fv(render.uniforms.at("material.ambient"), 1, material.ambient.data());
	glUniform3fv(render.uniforms.at("material.diffuse"), 1, material.diffuse.data());
	glUniform3fv(render.uniforms.at("material.specular"), 1, material.specular.data());
	glUniform1fv(render.uniforms.at("material.shininess"), 1, &material.shininess);
}