#include "Scene.hpp"
#include <iostream>

using namespace std;
using namespace Eigen;
using namespace cv;
using namespace PiratePhysics;

const Vector3f G{0.f, -9.8f, 0.f};

Scene::Scene() : SceneBase(), cube(25)
{
	loadResource();
	initCube();

	Vector3f origin = box1.getOrigin();
	origin[1] = 15.f;
	box1.setOrigin(origin);

	m_time = chrono::system_clock::now();
	dtAll = 0.f;
}

Scene::~Scene()
{
}

void Scene::update()
{
	auto timeNow = chrono::system_clock::now();
	std::chrono::duration<float> elapsed = timeNow - m_time;
	m_time = timeNow;
	float dt = elapsed.count();
	dt = 0.01f;
	dtAll += dt;

	physicsUpdate(dt);
	graphicsUpdate(dt);
}

void Scene::physicsUpdate(const float dt)
{
	Vector3f origin = box1.getOrigin() + box1.getVelocity() *dt;
	box1.setOrigin(origin);

	Vector3f rotation = box1.getOmega() * dt;
	Matrix3f rotationMatrix;
    rotationMatrix = AngleAxisf(rotation[0], Vector3f::UnitX())
                        * AngleAxisf(rotation[1],  Vector3f::UnitY())
                        * AngleAxisf(rotation[2], Vector3f::UnitZ());
    Matrix3f rotationChange = box1.getRotation() * rotationMatrix;
	box1.setRotation(rotationChange);


	auto penatrationDepth = collisionDetection(box1, box2);
	if(penatrationDepth)
	{
		Vector3f tt = penatrationDepth.value();
		collisionResolution(box1, box2, tt);
		cout << penatrationDepth.value() << endl;
	}

	Vector3f velocity = box1.getVelocity() + G*dt;
	box1.setVelocity(velocity);

	modelCube1.setIdentity();
	modelCube1.block<3, 3>(0, 0) = box1.getRotation();
	modelCube1.block<3, 1>(0, 3) = box1.getOrigin();

	modelCube2.setIdentity();
	modelCube2.block<3, 3>(0, 0) = box2.getRotation();
	modelCube2.block<3, 1>(0, 3) = box2.getOrigin();
}

void Scene::graphicsUpdate(const float dt)
{
	glUseProgram(shaderCube.Program);
	glUniform1i(shaderCube.uniforms.at("textureSampler"), 0);
	glUniformMatrix4fv(shaderCube.uniforms.at("projection"), 1, GL_FALSE, camera.projection.data());
	glUniformMatrix4fv(shaderCube.uniforms.at("view"), 1, GL_FALSE, camera.view.data());
	glUniformMatrix4fv(shaderCube.uniforms.at("model"), 1, GL_FALSE, modelCube1.data());

	glBindTexture(GL_TEXTURE_2D, texCube);
	glBindVertexArray(vaoCube);
	glDrawElements(GL_TRIANGLES, (GLsizei)cube.indices.size(), GL_UNSIGNED_INT, 0);

	glUniformMatrix4fv(shaderCube.uniforms.at("model"), 1, GL_FALSE, modelCube2.data());
	glDrawElements(GL_TRIANGLES, (GLsizei)cube.indices.size(), GL_UNSIGNED_INT, 0);

	glBindTexture(GL_TEXTURE_2D, 0);
	glBindVertexArray(0);
	glUseProgram(0);
}

void Scene::initCube()
{
	// vertex array
	glGenVertexArrays(1, &vaoCube);
	glBindVertexArray(vaoCube);

	// vertex buffer
	glGenBuffers(1, &vboCube);
	glBindBuffer(GL_ARRAY_BUFFER, vboCube);
	glBufferData(GL_ARRAY_BUFFER, sizeof(MeshBase::Vertex) * cube.data.size(), cube.data.data(), GL_DYNAMIC_DRAW);

	// index buffer
	glGenBuffers(1, &eboCube);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, eboCube);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned) * cube.indices.size(), cube.indices.data(), GL_STATIC_DRAW);

	// position attribute
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(MeshBase::Vertex), (GLvoid *)0);
	glEnableVertexAttribArray(0);

	// normal attribute
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(MeshBase::Vertex), (GLvoid *)(3 * sizeof(float)));
	glEnableVertexAttribArray(1);

	// texture attribute
	glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(MeshBase::Vertex), (GLvoid *)(6 * sizeof(float)));
	glEnableVertexAttribArray(2);

	// color attribute
	glVertexAttribPointer(3, 4, GL_FLOAT, GL_FALSE, sizeof(MeshBase::Vertex), (GLvoid *)(8 * sizeof(float)));
	glEnableVertexAttribArray(3);

	glBindVertexArray(0);

	glGenTextures(1, &texCube);
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, texCube);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, img.cols, img.rows, 0, GL_RGB, GL_UNSIGNED_BYTE, img.data);
	glGenerateMipmap(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, 0);

	// shader
	shaderCube.addShader("Solid.vert", GL_VERTEX_SHADER);
	shaderCube.addShader("Solid.frag", GL_FRAGMENT_SHADER);
	shaderCube.link();
	shaderCube.addUniform("projection");
	shaderCube.addUniform("view");
	shaderCube.addUniform("model");
	shaderCube.addUniform("textureSampler");
}

void Scene::loadResource()
{
	string filename = "container.jpg";
	img = imread(filename);
	cvtColor(img, img, COLOR_BGR2RGB);
}