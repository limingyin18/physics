#include "RigidBody.hpp"

using namespace PiratePhysics;

RigidBody::RigidBody(float m) : mMass{m}
{
    mInvMass = 1.f / mMass;
}

RigidBody::~RigidBody()
{
}