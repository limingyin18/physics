#pragma once

#include "MeshBasic.hpp"
#include "Camera.hpp"
#include "Shader.hpp"

class MeshBasicRender
{
public:
    MeshBasicRender() = default;
    MeshBasicRender(MeshBase*, Eigen::Matrix4f&, Camera*);
    void draw();
    void setMesh(MeshBase*);
    void resetVAO();
    void setPose(Eigen::Matrix4f& m){model=m;};
    void setCamera(Camera* cam){camera = cam;};

private:
    GLuint ebo, vbo, vao;
    Shader render;

    const Camera* camera;
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
    const MeshBase* meshBase;
};