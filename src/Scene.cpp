#include "Scene.hpp"
#include <iostream>

using namespace std;
using namespace Eigen;
using namespace cv;
using namespace PiratePhysics;
using namespace BasicGL;

const Vector3f G{0.f, -9.8f, 0.f};

Scene::Scene() : SceneBase(), cube(25)
{
	loadResource();
	loadModel();
	initCube();
	initPhysics();

	Vector3f origin = box1.getOrigin();
	origin[0] = 10.f;
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
	mPBD.dt = dt;
	for(auto &rb:mPBD.mRigiBodies)
	{
		rb.mVelocity[1] += -dt*rb.mInvMass*10.f;
		rb.mBaryCenter += dt*rb.mVelocity;
	}

	for(auto &c:mPBD.mConstraints)
	{
		c->solveConstraint();
	}

	// set geometry
	modelCube1.setIdentity();
	modelCube1.block<3, 3>(0, 0) = box1.getRotation();
	modelCube1.block<3, 1>(0, 3) = mPBD.mRigiBodies[0].mBaryCenter;

	modelCube2.setIdentity();
	modelCube2.block<3, 3>(0, 0) = box2.getRotation();
	modelCube2.block<3, 1>(0, 3) = mPBD.mRigiBodies[1].mBaryCenter;
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

	renderDragon.draw();
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

void Scene::initPhysics()
{
	mPBD.mRigiBodies.emplace_back(1.0f);
	mPBD.mRigiBodies.emplace_back(1.0f);
	mPBD.mRigiBodies[0].mInvMass = 0.f;

	mPBD.mRigiBodies[0].mBaryCenter = Vector3f(0.f, 0.f, 0.f);
	mPBD.mRigiBodies[0].mVelocity = Vector3f(0.f, 0.f, 0.f);
	mPBD.mRigiBodies[1].mBaryCenter = Vector3f(10.f, 0.f, 0.f);
	mPBD.mRigiBodies[1].mVelocity = Vector3f(0.f, 0.f, 0.f);

	mPBD.mConstraints.emplace_back(std::make_shared<Stretching>(mPBD, 0, 1, 10.f));
}

void Scene::loadResource()
{
	string filename = "container.jpg";
	img = imread(filename);
	cvtColor(img, img, COLOR_BGR2RGB);
}

void Scene::loadModel()
{
	string filename = "teapot.obj";
	vector<Loader::Vec3f> x;
	vector<MeshFaceIndices> faces;
	vector<Loader::Vec3f> normals;
    vector<Loader::Vec2f> texcoords;
	Loader::Vec3f scale{1.f, 1.f, 1.f};
	Loader::loadObj(filename, &x, &faces, &normals, &texcoords, scale);

	dragon.data.resize(x.size());
	auto f = [&x, &normals, &texcoords](unsigned i, MeshBase::Vertex&d){
		 d =MeshBase::Vertex(Eigen::Vector3f{x[i][0], x[i][1], x[i][2]});};
 	dragon.setVertices(f);
	dragon.recomputeNormals(dragon.data);
	dragon.indices.reserve(3*faces.size());
	for(auto &v:faces)
	{
		dragon.indices.push_back(static_cast<unsigned>(v.posIndices[0]));
		dragon.indices.push_back(static_cast<unsigned>(v.posIndices[1]));
		dragon.indices.push_back(static_cast<unsigned>(v.posIndices[2]));
	}

	renderDragon.setMesh(&dragon);
	renderDragon.setCamera(&camera);
}