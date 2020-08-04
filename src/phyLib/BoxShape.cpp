#include <algorithm>
#include "BoxShape.hpp"

using namespace std;
using namespace Eigen;

BoxShape::BoxShape(Vector3f len, Vector3f origin, Matrix3f rot, float den) :
    PolyhedralConvexShape{origin, rot}, m_len{len}, m_density{den}
{
    m_mass = m_len.prod()*m_density;
    Matrix3f inertia;
    inertia << powf(m_len(1), 2)+powf(m_len(2), 2), 0.f, 0.f,
                   0.f, powf(m_len(0), 2)+powf(m_len(2), 2), 0.f,
                   0.f, 0.f, powf(m_len(0), 2)+powf(m_len(1), 2);
    inertia *= (m_mass / 3.f);
    mInertiaInv = inertia.inverse();

    mVelocity = {0.f, 0.f, 0.f};
    mOmega = {0.f, 0.f, 0.f};
}

BoxShape::~BoxShape()
{
}

std::pair<Vector3f, Vector3f> BoxShape::getAabb() const
{
    return {m_origin - m_rot*m_len, m_origin + m_rot*m_len};
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
    return {m_len[0]*pos1, m_len[1]*pos2, m_len[2]*pos3};
}