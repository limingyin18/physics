#pragma once

#include <array>
#include <iostream>
#include <Eigen/Eigen>
#include <Eigen/Geometry>

class Camera
{
public:
    float fov = 3.14f / 4.0f;
    float aspect = 4.0f/3.0f;
    float near = 0.1f;
    float far = 1000.0f;

    float targetDistance = 25.0f;
    Eigen::Vector3f target = {0.0f, 0.0f, 0.0f};
    Eigen::Vector3f rotation ={0.0f, 0.0f, 0.0f};

    Eigen::Matrix4f view;
    Eigen::Matrix4f projection;

    void computeTransformation();

public:
    explicit Camera();
    ~Camera() = default;

    Eigen::Vector3f getCameraPos() const;
};