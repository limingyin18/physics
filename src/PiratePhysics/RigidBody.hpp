#pragma once

#include <vector>
#include <Eigen/Eigen>

using namespace Eigen;

namespace PiratePhysics
{
    /**
     * Rogid body
     */
    class RigidBody
    {
    public:
        float mMass;
        float mInvMass;

        Vector3f mVelocity;
        Vector3f mBaryCenter;
    public:
        RigidBody(float m);
        ~RigidBody();
    };
}
