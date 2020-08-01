#include "CollisionShape.hpp"

using namespace Eigen;

CollisionShape::CollisionShape(Vector3f origin, Matrix3f rot) :
    m_origin{origin}, m_rot{rot}
{

}

Vector3f CollisionShape::getCenter() const
{
    return m_origin;
}

Matrix4f CollisionShape::getTransform() const
{
    Matrix4f trans; // transformation matrix
    trans.setIdentity();   // set to identity
    trans.block<3,3>(0,0) = m_rot; // first 3x3 block set to rotation matrix
    trans.block<3,1>(0,3) = m_origin; // fourth column set to translation vector
    return trans;
}