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

class Scene : public SceneBase
{
public:
    explicit Scene();
    ~Scene();

    void update() override;

    void physicsUpdate(const float dt);
    void graphicsUpdate(const float dt);

private:
    void initCube();
    void initPhysics();
    void loadResource();
    void loadModel();

private:
    std::chrono::time_point<std::chrono::system_clock> m_time;
    float dtAll;

    PiratePhysics::BoxShape box1, box2;

    PiratePhysics::PositionBasedDynamics mPBD;

    Eigen::Matrix4f modelCube1, modelCube2;

    Cube cube;
    Shader shaderCube;
    unsigned eboCube, vboCube, vaoCube;
    unsigned texCube;

    cv::Mat img;

    MeshBase dragon;
    MeshBasicRender renderDragon;
};