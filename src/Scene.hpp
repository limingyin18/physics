#pragma once

#include <chrono>
#include <array>
#include <random>
#include <opencv2/opencv.hpp>
#include "basic/SceneBase.hpp"
#include "basic/MeshBasic.hpp"
#include "basic/Shader.hpp"
#include "phyLib/BoxShape.hpp"
#include "phyLib/ConvexCollision.hpp"

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
    void loadResource();

private:
    std::chrono::time_point<std::chrono::system_clock> m_time;
    float dtAll;

    BoxShape box1, box2;

    Eigen::Matrix4f modelCube1, modelCube2;

    Cube cube;
    Shader shaderCube;
    unsigned eboCube, vboCube, vaoCube;
    unsigned texCube;

    cv::Mat img;
};