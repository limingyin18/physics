#pragma once

#include <vector>
#include <memory>
#include "RigidBody.hpp"

namespace PiratePhysics
{
    class Constraint;

    class PositionBasedDynamics
    {
    public:
        std::vector<std::shared_ptr<Constraint>> mConstraints;
        std::vector<RigidBody> mRigiBodies;
        float dt;
    public:
        PositionBasedDynamics(/* args */);
        ~PositionBasedDynamics();
    };
}