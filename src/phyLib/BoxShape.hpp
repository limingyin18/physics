#pragma once

#include <Eigen/Eigen>
#include "PolyhedralConvexShape.hpp"

/** 
 * @brief boxShape is a class for box shape
 */
class BoxShape final : public PolyhedralConvexShape
{
public:
    Eigen::Vector3f m_len; // length of half side

    float m_density; // density
    float m_mass; // mass
    Eigen::Vector3f m_inertia; // inertia

public:
    BoxShape(Eigen::Vector3f len={1.f, 1.f, 1.f}, 
        Eigen::Vector3f origin={0.f, 0.f, 0.f},
        Eigen::Matrix3f rot=Eigen::Vector3f{1.f, 1.f, 1.f}.asDiagonal(),
        float den = 1.0f);
    ~BoxShape();

	std::pair<Eigen::Vector3f, Eigen::Vector3f> getAabb() const override;

   	virtual int getNumVertices() const override;
	virtual Eigen::Vector3f getVertex(size_t index) const override;
};