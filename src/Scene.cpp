#include "Scene.hpp"
#include <iostream>

using namespace std;
using namespace Eigen;
using namespace cv;
using namespace PiratePhysics;
using namespace BasicGL;

const Vector3f G{0.f, -9.8f, 0.f};
const Vector3f LIGHT_AMBIENT = {0.2f, 0.2f, 0.2f};
const Vector3f LIGHT_DIFFUSE = {0.5f, 0.5f, 0.5f};
const Vector3f LIGHT_SPECULAR = {1.f, 1.0f, 1.0f};
const Vector3f LIGHT_POSITION{5.0f, 5.0f, 0.0f};
string TeapotFileName = "teapot.obj";

Scene::Scene() : SceneBase(), cube(25), light(25)
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
	renderTeapot.draw();

	renderCube.setPose(modelCube1);
	renderCube.draw();

	renderCube.setPose(modelCube2);
	renderCube.draw();

	renderLight.setPose(modelLight);
	renderLight.draw();
}

void Scene::initCube()
{
 	light.setVertices([](unsigned i, MeshBase::Vertex&d){d.position *= 0.5f;d.color = LIGHT_SPECULAR;});

	modelLight.setIdentity();
	modelLight.block<3, 1>(0, 3) = LIGHT_POSITION;
	renderLight.setMesh(&light);
	renderLight.setCamera(&camera);

	renderCube.setMesh(&cube);
	renderCube.setCamera(&camera);
	renderCube.setTexImg(img.cols, img.rows, img.data);
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
	vector<Loader::Vec3f> x;
	vector<MeshFaceIndices> faces;
	vector<Loader::Vec3f> normals;
    vector<Loader::Vec2f> texcoords;
	Loader::Vec3f scale{1.f, 1.f, 1.f};
	Loader::loadObj(TeapotFileName, &x, &faces, &normals, &texcoords, scale);

	teapot.data.resize(x.size());
	auto f = [&x, &normals, &texcoords](unsigned i, MeshBase::Vertex&d){
		 d =MeshBase::Vertex(Eigen::Vector3f{x[i][0], x[i][1], x[i][2]});
		 d.color = Eigen::Vector3f{1.0f, 1.0f, 1.0f};};
 	teapot.setVertices(f);
	teapot.indices.reserve(3*faces.size());
	teapot.indices.clear();
	for(auto &v:faces)
	{
		teapot.indices.push_back(static_cast<unsigned>(v.posIndices[0] - 1));
		teapot.indices.push_back(static_cast<unsigned>(v.posIndices[1] - 1));
		teapot.indices.push_back(static_cast<unsigned>(v.posIndices[2] - 1));
	}
	teapot.recomputeNormals(teapot.data);

	MeshBasicRender::Material materialTeapot;
	materialTeapot.ambient = {1.0f, 0.5f, 0.3f};
	materialTeapot.diffuse = {1.0f, 0.5f, 0.3f};
	materialTeapot.specular = {0.5f, 0.5f, 0.5f};
	materialTeapot.shininess = 32.0f;
	renderTeapot.setMaterial(materialTeapot);
	renderTeapot.setMesh(&teapot);
	MeshBasicRender::Light light;
	light.position = LIGHT_POSITION;
	light.ambient = LIGHT_AMBIENT;
	light.diffuse = LIGHT_DIFFUSE;
	light.specular = LIGHT_SPECULAR;
	renderTeapot.setLight(light);
	renderTeapot.setCamera(&camera);
}