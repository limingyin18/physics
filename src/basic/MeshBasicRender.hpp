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


    void setMesh(MeshBase*);
    void setTexImg(int, int, unsigned char*);
    void setPose(Eigen::Matrix4f& m){model=m;};
    void setCamera(Camera* cam){camera = cam;};

protected:
    virtual void setShader();
    virtual void setVAO();
    virtual void bindShader();
    virtual void _draw();
    virtual void unBindShader();

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

protected:
    void setShader() override;
    void bindShader() override;
    void unBindShader() override;
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

protected:
    void setShader() override;
};

/**
 * @brief phong render
 * 
 */
class PhongRender : public MeshBasicRender
{
public:
    PhongRender() = default;
    virtual ~PhongRender() = default;

protected:
    void setShader() override;
    void bindShader() override;
};