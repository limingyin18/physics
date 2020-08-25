#pragma once

#include <utility>
#include <Eigen/Eigen>

namespace PiratePhysics
{
 /**
  * @brief abstract class of collision shape
  */
class CollisionShape
{
protected:
	float mDensity;
    float mMassInv;
    Eigen::Matrix3f mInertiaInv;

    Eigen::Vector3f mVelocity; // linear velocity
    Eigen::Vector3f mOmega; // angular velocity
    Eigen::Vector3f mOrigin; // position
    Eigen::Matrix3f mRot; // rotation 

public:
	CollisionShape(const Eigen::Vector3f &origin={0.f, 0.f, 0.f},
        const Eigen::Matrix3f &rot=Eigen::Vector3f{1.f, 1.f, 1.f}.asDiagonal(), const Eigen::Vector3f &velocity={0.f, 0.f, 0.f},
		const Eigen::Vector3f &omega={0.f, 0.f, 0.f}, const float density=1.0f);
   virtual  ~CollisionShape(){};

	virtual std::pair<Eigen::Vector3f, Eigen::Vector3f> getAabb() const = 0;
	virtual int getNumVertices() const = 0;
	virtual Eigen::Vector3f getVertex(size_t) const = 0;
    Eigen::Vector3f localGetSupportingVertex(Eigen::Vector3f dir) const;


	float getMassInv() const;
	Eigen::Matrix3f getInertiaInv() const;

	Eigen::Vector3f getVelocity() const;
	void setVelocity(Eigen::Vector3f &);

	Eigen::Vector3f getOmega() const;
	void setOmega(Eigen::Vector3f &);

	Eigen::Vector3f getOrigin() const;
	void setOrigin(Eigen::Vector3f &);

	Eigen::Matrix3f getRotation() const;
	void setRotation(Eigen::Matrix3f &);

	Eigen::Matrix4f getTransform() const;
};
}