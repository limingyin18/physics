#include "MeshBasicRender.hpp"

using namespace Eigen;

void MeshBasicRender::draw()
{
	glUseProgram(render.Program);
	glUniformMatrix4fv(render.uniforms.at("projection"), 1, GL_FALSE, camera->projection.data());
	glUniformMatrix4fv(render.uniforms.at("view"), 1, GL_FALSE, camera->view.data());
	glUniformMatrix4fv(render.uniforms.at("model"), 1, GL_FALSE, model.data());
	glBindVertexArray(vao);
	glDrawElements(GL_TRIANGLES, (GLsizei)meshBase->indices.size(), GL_UNSIGNED_INT, 0);
}

void MeshBasicRender::setMesh(MeshBase* m)
{
	meshBase = m;
	setVAO();
	setShader();
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

void SolidRender::setVAO()
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
	glVertexAttribPointer(3, 4, GL_FLOAT, GL_FALSE, sizeof(MeshBase::Vertex), (GLvoid *)(8 * sizeof(float)));
	glEnableVertexAttribArray(3);
}

void SolidRender::setShader()
{
    render.addShader("Solid.vert", GL_VERTEX_SHADER);
    render.addShader("Solid.frag", GL_FRAGMENT_SHADER);
    render.link();
	render.addUniform("projection");
	render.addUniform("view");
	render.addUniform("model");
	render.addUniform("textureSampler");
}

void SolidRender::draw()
{
	glUseProgram(render.Program);
	glUniformMatrix4fv(render.uniforms.at("projection"), 1, GL_FALSE, camera->projection.data());
	glUniformMatrix4fv(render.uniforms.at("view"), 1, GL_FALSE, camera->view.data());
	glUniformMatrix4fv(render.uniforms.at("model"), 1, GL_FALSE, model.data());
	glBindVertexArray(vao);

	glBindTexture(GL_TEXTURE_2D, tex);

	glDrawElements(GL_TRIANGLES, (GLsizei)meshBase->indices.size(), GL_UNSIGNED_INT, 0);
}

void CeramicRender::setVAO()
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
	glVertexAttribPointer(3, 4, GL_FLOAT, GL_FALSE, sizeof(MeshBase::Vertex), (GLvoid *)(8 * sizeof(float)));
	glEnableVertexAttribArray(3);;
}

void CeramicRender::setShader()
{
    render.addShader("ceramic.vert", GL_VERTEX_SHADER);
    render.addShader("ceramic.frag", GL_FRAGMENT_SHADER);
    render.link();
	render.addUniform("projection");
	render.addUniform("view");
	render.addUniform("model");
}

void CeramicRender::draw()
{
	glUseProgram(render.Program);
	glUniformMatrix4fv(render.uniforms.at("projection"), 1, GL_FALSE, camera->projection.data());
	glUniformMatrix4fv(render.uniforms.at("view"), 1, GL_FALSE, camera->view.data());
	glUniformMatrix4fv(render.uniforms.at("model"), 1, GL_FALSE, model.data());
	glBindVertexArray(vao);

	glDrawElements(GL_TRIANGLES, (GLsizei)meshBase->indices.size(), GL_UNSIGNED_INT, 0);
}