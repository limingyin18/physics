#include "MeshBasicRender.hpp"

using namespace Eigen;

MeshBasicRender::MeshBasicRender(MeshBase* o, Matrix4f& m, Camera* cam) : meshBase{o}, camera{cam}, model{m}
{
	resetVAO();
}

void MeshBasicRender::resetVAO()
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

	// color attribute
	glVertexAttribPointer(2, 4, GL_FLOAT, GL_FALSE, sizeof(MeshBase::Vertex), (GLvoid*)(6 * sizeof(float)));
	glEnableVertexAttribArray(2);

   // shader
    render.addShader("Solid.vert", GL_VERTEX_SHADER);
    render.addShader("Solid.frag", GL_FRAGMENT_SHADER);
    render.link();
	render.addUniform("projection");
	render.addUniform("view");
	render.addUniform("model");
}

void MeshBasicRender::setMesh(MeshBase* m)
{
	meshBase = m;
	resetVAO();
}

void MeshBasicRender::draw()
{
	glUseProgram(render.Program);
	glUniformMatrix4fv(render.uniforms.at("projection"), 1, GL_FALSE, camera->projection.data());
	glUniformMatrix4fv(render.uniforms.at("view"), 1, GL_FALSE, camera->view.data());
	glUniformMatrix4fv(render.uniforms.at("model"), 1, GL_FALSE, model.data());
	glBindVertexArray(vao);
	glDrawElements(GL_TRIANGLES, (GLsizei)meshBase->indices.size(), GL_UNSIGNED_INT, 0);
}