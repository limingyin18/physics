#pragma once

#include <Eigen/Eigen>
#include "CollisionShape.hpp"

namespace PiratePhysics
{
/** 
 * @brief boxShape is a class for box shape
 */
class BoxShape final : public CollisionShape
{
public:
    Eigen::Vector3f mLength; // length of half side
 public:
    BoxShape(const Eigen::Vector3f &len={1.f, 1.f, 1.f}, 
        const Eigen::Vector3f &origin={0.f, 0.f, 0.f},
        const Eigen::Matrix3f &rot=Eigen::Vector3f{1.f, 1.f, 1.f}.asDiagonal(),
        const Eigen::Vector3f &velocity={0.f, 0.f, 0.f},
        const Eigen::Vector3f &omega={0.f, 0.f, 0.f},
        float den = 1.0f);
    ~BoxShape();

	std::pair<Eigen::Vector3f, Eigen::Vector3f> getAabb() const override;

   	virtual int getNumVertices() const override;
	virtual Eigen::Vector3f getVertex(size_t index) const override;
};
}