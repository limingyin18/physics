#pragma once

#include "MeshBasic.hpp"
#include "Camera.hpp"
#include "Shader.hpp"

/**
 * @brief basic render interface
 * 
 */
class MeshBasicRender
{
public:
    MeshBasicRender() = default;
    MeshBasicRender(MeshBase*, Eigen::Matrix4f&, Camera*);
    virtual ~MeshBasicRender() = default;
    virtual void draw();
    virtual void setVAO() = 0;
    virtual void setShader() = 0;
    void setMesh(MeshBase*);
    void setTexImg(int, int, unsigned char*);
    void setPose(Eigen::Matrix4f& m){model=m;};
    void setCamera(Camera* cam){camera = cam;};

protected:
    GLuint ebo, vbo, vao;
    GLuint tex;
    Shader render;

    const Camera* camera;
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
    const MeshBase* meshBase;
};

/**
 * @brief solid render
 * 
 */
class SolidRender : public MeshBasicRender
{
public:
    SolidRender() = default;
    virtual ~SolidRender() = default;

    void setVAO() override;
    void setShader() override;
    void draw() override;
};

/**
 * @brief ceramic render
 * 
 */
class CeramicRender : public MeshBasicRender
{
public:
    CeramicRender() = default;
    virtual ~CeramicRender() = default;

    void setVAO() override;
    void setShader() override;

    void draw() override;
};