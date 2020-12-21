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

Scene::Scene() : SceneBase(), cube(25), light(25), vox1(2)
{
	initTeapot();
	initCube();
	initLight();

	initPhysics();
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
	for (auto &rb : mPBD.mRigiBodies)
	{
		rb.mVelocity[1] += -dt * rb.mInvMass * 10.f;
		rb.mBaryCenter += dt * rb.mVelocity;
	}

	for (auto &c : mPBD.mConstraints)
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

	for(auto &m:modelVoxs)
	{
		renderVox.setPose(m);
		renderVox.draw();
	}
}

void Scene::initLight()
{
	light.setVertices([](unsigned i, MeshBase::Vertex &d) {d.position *= 0.5f;d.color = LIGHT_SPECULAR; });

	modelLight.setIdentity();
	modelLight.block<3, 1>(0, 3) = LIGHT_POSITION;
	renderLight.setMesh(&light);
	renderLight.setCamera(&camera);
	renderLight.setRender();
}

void Scene::initCube()
{
	// load texture
	string filename = "container.jpg";
	img = imread(filename);
	cvtColor(img, img, COLOR_BGR2RGB);

	renderCube.setTexImg(img.cols, img.rows, img.data);
	renderCube.setMesh(&cube);
	renderCube.setCamera(&camera);
	renderCube.setRender();
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

void Scene::initTeapot()
{
	// load model
	vector<Loader::Vec3f> x;
	vector<MeshFaceIndices> faces;
	vector<Loader::Vec3f> normals;
	vector<Loader::Vec2f> texcoords;
	Loader::Vec3f scale{1.f, 1.f, 1.f};
	Loader::loadObj(TeapotFileName, &x, &faces, &normals, &texcoords, scale);

	// feed data to mesh struct
	teapot.data.resize(x.size());
	auto f = [&x, &normals, &texcoords](unsigned i, MeshBase::Vertex &d) {
		 d =MeshBase::Vertex(Eigen::Vector3f{x[i][0], x[i][1], x[i][2]});
		 d.color = Eigen::Vector3f{1.0f, 1.0f, 1.0f}; };
	teapot.setVertices(f);
	teapot.indices.reserve(3 * faces.size());
	teapot.indices.clear();
	for (auto &v : faces)
	{
		teapot.indices.push_back(static_cast<unsigned>(v.posIndices[0] - 1));
		teapot.indices.push_back(static_cast<unsigned>(v.posIndices[1] - 1));
		teapot.indices.push_back(static_cast<unsigned>(v.posIndices[2] - 1));
	}
	teapot.recomputeNormals(teapot.data);

	// set model render
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
	renderTeapot.setRender();

	vector<Vector3f> testVertices(x.size());
	for (size_t i = 0; i < x.size(); ++i)
	{
		testVertices[i][0] = x[i][0];
		testVertices[i][1] = x[i][1];
		testVertices[i][2] = x[i][2];
	}

	Eigen::Vector3f maxExtents(-numeric_limits<float>::max(),
							   -numeric_limits<float>::max(),
							   -numeric_limits<float>::max());

	Eigen::Vector3f minExtents(numeric_limits<float>::max(),
							   numeric_limits<float>::max(),
							   numeric_limits<float>::max());
	for (auto &v : testVertices)
	{
		maxExtents[0] = maxExtents[0] > v[0] ? maxExtents[0] : v[0];
		maxExtents[1] = maxExtents[1] > v[1] ? maxExtents[1] : v[1];
		maxExtents[2] = maxExtents[2] > v[2] ? maxExtents[2] : v[2];

		minExtents[0] = minExtents[0] < v[0] ? minExtents[0] : v[0];
		minExtents[1] = minExtents[1] < v[1] ? minExtents[1] : v[1];
		minExtents[2] = minExtents[2] < v[2] ? minExtents[2] : v[2];
	}
	unsigned width = 20;
	unsigned height = 20;
	unsigned depth = 20;
	vector<unsigned> volume;
	Voxelize(testVertices.data(), testVertices.size(),
			 teapot.indices.data(), faces.size(), width, height,
			 depth, volume, minExtents, maxExtents);

	Vector3f extents(maxExtents - minExtents);
	Vector3f delta(extents[0] / width, extents[1] / height, extents[2] / depth);
	Vector3f offset(0.5f * delta[0], 0.5f * delta[1], 0.5f * delta[2]);
	for(size_t x = 0; x < width; ++x)
	{
		for(size_t y = 0; y < height; ++y)
		{
			for(size_t z = 0; z < depth; ++z)
			{
				if(volume[z * width * height + y * width + x] == 1)
				{
					Matrix4f modelVoxT;
					modelVoxT.setIdentity();
					Vector3f pos = minExtents + Vector3f(x * delta[0] + offset[0], 
														 y * delta[1] + offset[1] + 3.0f,
														 z * delta[2] + offset[2]);
					modelVoxT.block<3, 1>(0, 3) = pos;
					modelVoxs.push_back(modelVoxT);
				}
			}
		}
	}
	delta = delta * 0.5f * 0.8f;
	vox1.setVertices([&delta](unsigned i, MeshBase::Vertex &d) {d.position[0] *= delta[0];
																d.position[1] *= delta[1];
																d.position[2] *= delta[2];});
	renderVox.setCamera(&camera);
	renderVox.setMesh(&vox1);
	renderVox.setRender();
}