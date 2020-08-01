#pragma once

#include <utility>
#include <Eigen/Eigen>

///abstract interface class for collision shape
class CollisionShape
{
public:
	CollisionShape(Eigen::Vector3f origin={0.f, 0.f, 0.f},
        Eigen::Matrix3f rot=Eigen::Vector3f{1.f, 1.f, 1.f}.asDiagonal());
   virtual  ~CollisionShape(){};

	///get the axis aligned bounding box 
	virtual std::pair<Eigen::Vector3f, Eigen::Vector3f> getAabb() const = 0;
	virtual int getNumVertices() const = 0;
	virtual Eigen::Vector3f getVertex(size_t index) const = 0;
	Eigen::Vector3f getCenter() const;
	Eigen::Matrix4f getTransform() const;

public:
    Eigen::Vector3f m_origin; // translation
    Eigen::Matrix3f m_rot; // rotation 
};