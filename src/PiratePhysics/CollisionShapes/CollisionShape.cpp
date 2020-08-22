#include "CollisionShape.hpp"

using namespace std;
using namespace Eigen;
using namespace PiratePhysics;

constexpr float EPSILON = 1e-6;

CollisionShape::CollisionShape(const Vector3f &origin, const Matrix3f &rot, const Vector3f &velocity,
    const Vector3f &omega, const float density) :
    mOrigin{origin}, mRot{rot}, mVelocity{velocity}, mOmega{omega}, mDensity{density}
{
}

Eigen::Vector3f CollisionShape::getVelocity() const
{
	return mVelocity;
}

void CollisionShape::setVelocity(Eigen::Vector3f &velocity)
{
	mVelocity = velocity;
}

Eigen::Vector3f CollisionShape::getOmega() const
{
	return mOmega;
}

void CollisionShape::setOmega(Eigen::Vector3f &omega)
{
	mOmega = omega;
}

Vector3f CollisionShape::getOrigin() const
{
    return mOrigin;
}

void CollisionShape::setOrigin(Eigen::Vector3f &origin)
{
    mOrigin = origin;
}

Matrix3f CollisionShape::getRotation() const
{
    return mRot;
}

void CollisionShape::setRotation(Eigen::Matrix3f &rotation)
{
    mRot = rotation;
}

Matrix4f CollisionShape::getTransform() const
{
    Matrix4f trans; // transformation matrix
    trans.setIdentity();   // set to identity
    trans.block<3,3>(0,0) = mRot; // first 3x3 block set to rotation matrix
    trans.block<3,1>(0,3) = mOrigin; // fourth column set to translation vector
    return trans;
}

Vector3f CollisionShape::localGetSupportingVertex(Vector3f dir) const
{
	if (dir.norm() < EPSILON)
	{
		dir = Vector3f{1.f, 0.f, 0.f};
	}
	else
	{
       dir.normalize();
	}

    Vector3f supVertex(0.f, 0.f, 0.f);
	float tempDot = -numeric_limits<float>::max();
	Matrix4f trans = getTransform();
	for (size_t i = 0; i < getNumVertices(); i++)
	{
        Vector4f tempVertexH = {1.0f, 1.0f, 1.0f, 1.0f};
		Vector3f tempVertex = getVertex(i);
		tempVertexH[0] = tempVertex[0];
		tempVertexH[1] = tempVertex[1];
		tempVertexH[2] = tempVertex[2];
		tempVertexH = trans * tempVertexH;
		Vector3f vertex = {tempVertexH[0] / tempVertexH[3], 
						   tempVertexH[1] / tempVertexH[3],
						   tempVertexH[2] / tempVertexH[3]};
        float newDot = vertex.dot(dir);
		if (newDot > tempDot)
		{
			tempDot = newDot;
			supVertex = vertex;
		}
	}

	return supVertex;
}