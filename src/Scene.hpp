#pragma once

#include <chrono>
#include <array>
#include <random>
#include <opencv2/opencv.hpp>
#include "basic/SceneBase.hpp"
#include "basic/MeshBasic.hpp"
#include "basic/Shader.hpp"
#include "basic/Loader.hpp"
#include "basic/MeshBasicRender.hpp"
#include "PiratePhysics/CollisionShapes/BoxShape.hpp"
#include "PiratePhysics/ConvexCollision.hpp"
#include "PiratePhysics/CollisionResolution.hpp"
#include "PiratePhysics/PositionBasedDynamics.hpp"
#include "PiratePhysics/Constraint.hpp"
#include "PiratePhysics/AABBTree.hpp"

class Scene : public SceneBase
{
public:
    explicit Scene();
    ~Scene();

    void update() override;

    void physicsUpdate(const float dt);
    void graphicsUpdate(const float dt);

private:
    void initLight();
    void initCube();
    void initTeapot();

    void initPhysics();

private:
    std::chrono::time_point<std::chrono::system_clock> m_time = 
        std::chrono::system_clock::now();
    float dtAll = 0.0f;

    Cube light;
    MeshBasicRender renderLight;
    Eigen::Matrix4f modelLight;

    Eigen::Matrix4f modelCube1, modelCube2;

    PiratePhysics::BoxShape box1, box2;
    PiratePhysics::PositionBasedDynamics mPBD;

    Cube cube;
    SolidRender renderCube;
    cv::Mat img;

    MeshBase teapot;
    PhongRender renderTeapot;
};