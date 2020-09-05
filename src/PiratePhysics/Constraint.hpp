#pragma once

#include <vector>
#include <Eigen/Eigen>
#include "PositionBasedDynamics.hpp"

namespace PiratePhysics
{
    class Constraint
    {
    public:
        PositionBasedDynamics &mPBD;
        std::vector<int> mBodyIndexes;

    public:
        Constraint(PositionBasedDynamics &);
        virtual ~Constraint();

        virtual void solveConstraint() = 0;
    };
    

    
    class Stretching : public Constraint
    {
    public:
        Stretching(PositionBasedDynamics &, int rbI0, int rbI1, float d);
        virtual ~Stretching();

        float distance;

        void solveConstraint() override;
    };
}