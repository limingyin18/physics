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

    class ShapeMatchingConstraint : public Constraint
    {
    public:
        ShapeMatchingConstraint(PositionBasedDynamics &, const unsigned num,
            const std::vector<unsigned> &inds);
        virtual ~ShapeMatchingConstraint();

        void solveConstraint() override;

        Vector3f mRestCM;
        Matrix3f mInvRestMat;
        std::vector<Eigen::Vector3f> mX0;
        std::vector<Eigen::Vector3f> mX;
        std::vector<Eigen::Vector3f> mCorr;
        std::vector<Eigen::Vector3f> mNumClusters;
        std::vector<float> mW;
        std::vector<unsigned> indices;
    };
}