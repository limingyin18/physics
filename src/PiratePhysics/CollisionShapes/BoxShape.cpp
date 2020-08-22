#include <algorithm>
#include "BoxShape.hpp"

using namespace std;
using namespace Eigen;
using namespace PiratePhysics;

BoxShape::BoxShape(const Vector3f &len, const Vector3f &origin, const Matrix3f &rot,
    const Vector3f &velocity, const Vector3f &omega, float den) :
    CollisionShape{origin, rot, velocity, omega, den}, mLength{len}
{
    mMassInv = 1.f / (mLength.prod()*mDensity);

    Matrix3f inertia;
    inertia << powf(mLength(1), 2)+powf(mLength(2), 2), 0.f, 0.f,
                   0.f, powf(mLength(0), 2)+powf(mLength(2), 2), 0.f,
                   0.f, 0.f, powf(mLength(0), 2)+powf(mLength(1), 2);
    inertia *= 1.f / 3.f;
    mInertiaInv = inertia.inverse() * mMassInv;
}

BoxShape::~BoxShape()
{
}

std::pair<Vector3f, Vector3f> BoxShape::getAabb() const
{
    return {mOrigin - mRot*mLength, mOrigin + mRot*mLength};
}

int BoxShape::getNumVertices() const 
{
    return 8;
}

Vector3f BoxShape::getVertex(size_t index) const 
{
    int index_i = static_cast<int>(index);
    float pos1 = static_cast<float>((index_i & 1) * 2 - 1);
    float pos2 = static_cast<float>(((index_i & 2) >> 1) * 2 - 1);
    float pos3 = static_cast<float>(((index_i & 4) >> 2) * 2 - 1);
    return {mLength[0]*pos1, mLength[1]*pos2, mLength[2]*pos3};
}