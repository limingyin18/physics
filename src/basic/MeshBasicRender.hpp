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

    void setRender();

    /**
     * @brief material's ambient diffuse specular and shiness cofficient
     * 
     */
    struct Material
    {
        Material() = default;
        Material(const Material&) = default;
        Material& operator=(const Material&) = default;
        Eigen::Vector3f ambient = {1.f, 1.f, 1.f};
        Eigen::Vector3f diffuse = {1.f, 1.f, 1.f};
        Eigen::Vector3f specular = {1.f, 1.f, 1.f};
        float shininess = 1.0f;
    };

    /**
     * @brief light's position, ambient, diffuse, specular
     * 
     */
    struct Light
    {
        Light() = default;
        Light(const Light&) = default;
        Light& operator=(const Light&) = default;
        Eigen::Vector3f position = {0.f, 0.f, 0.f};
        Eigen::Vector3f ambient = {1.f, 1.f, 1.f};
        Eigen::Vector3f diffuse = {1.f, 1.f, 1.f};
        Eigen::Vector3f specular = {1.f, 1.f, 1.f};
    };

protected:
    /**
     * @brief add shader files and link
     * 
     */
    virtual void addShader();

    /**
     * @brief link shader
     * 
     */
    virtual void linkShader();

    /**
     * @brief add uniforms: mvp matrixs
     * 
     */
    virtual void addUniforms();

    virtual void setVAO();

    // draw
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
    void addShader() override;
    void addUniforms() override;
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
    void addShader() override;
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

    void setMaterial(const Material &m){material = m;};
    void setLight(const Light &l){light = l;};

protected:
    void addShader() override;
    void addUniforms() override;
    void bindShader() override;

    Material material;
    Light light;
};