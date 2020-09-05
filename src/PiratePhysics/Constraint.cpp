#include "Constraint.hpp"
#include "RigidBody.hpp"

using namespace std;
using namespace Eigen;
using namespace PiratePhysics;

Constraint::Constraint(PositionBasedDynamics &pbd) : mPBD{pbd}
{
}

Constraint::~Constraint()
{
}

Stretching::Stretching(PositionBasedDynamics &pbd, int rbI0, int rbI1, float d) : Constraint(pbd)
{
    mBodyIndexes.push_back(rbI0);
    mBodyIndexes.push_back(rbI1);
    distance = d;
}

Stretching::~Stretching()
{
}

void Stretching::solveConstraint()
{
    RigidBody &rb0 = mPBD.mRigiBodies[mBodyIndexes[0]];
    RigidBody &rb1 = mPBD.mRigiBodies[mBodyIndexes[1]];

    Vector3f x0x1 = rb0.mBaryCenter - rb1.mBaryCenter;
    float x0x1N = x0x1.norm();
    float lambda = -(x0x1N - distance) / (rb0.mInvMass + rb1.mInvMass);

    Vector3f deltaX0 = rb0.mInvMass * lambda / x0x1N * x0x1;
    Vector3f deltaX1 = -rb1.mInvMass * lambda / x0x1N * x0x1;

    rb0.mVelocity += deltaX0 / mPBD.dt;
    rb1.mVelocity += deltaX1 / mPBD.dt;
    rb0.mBaryCenter += deltaX0;
    rb1.mBaryCenter += deltaX1;
}