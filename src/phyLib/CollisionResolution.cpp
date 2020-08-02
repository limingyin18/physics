#include "CollisionResolution.hpp"

void makeOrthonormalBasis(const Eigen::Vector3f &x, Eigen::Vector3f &y, Eigen::Vector3f &z)
{
    // calculate z from the vector product of x and y.
    z = x.cross(y);

    // check for y and z in parallel
    if (z.squaredNorm() == 0.0f)
        return;

    // calculate y from the vector product of z and x.
    y = z.cross(x);

    // normalize the output vectors
    y.normalize();
    z.normalize();
}