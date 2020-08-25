#include "CollisionResolution.hpp"

using namespace Eigen;
using namespace PiratePhysics;

namespace PiratePhysics
{
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

void collisionResolution(PiratePhysics::CollisionShape &a, PiratePhysics::CollisionShape&b, Eigen::Vector3f &penatration)
{
    // contact point
    Vector3f contactPoint = a.localGetSupportingVertex(penatration);

    // contact world
    Vector3f contactNormal = -penatration.normalized(), y, z;
    if(abs(penatration[0]) > abs(penatration[1]))
    {
        y = {0.f, 1.0f, 0.0f};
    }
    else
    {
        y = {1.0f, 0.0f, 1.0f};
    }
    makeOrthonormalBasis(contactNormal, y, z);
    Matrix3f contactWorld;
    contactWorld.block<3, 1>(0, 0) = contactNormal;
    contactWorld.block<3, 1>(0, 1) = y;
    contactWorld.block<3, 1>(0, 2) = z;

    // velocity change per impluse
    Vector3f linearVelPer = Vector3f{1.0f, 1.0f, 1.0f}*a.getMassInv(); 
    Vector3f positionRel = contactPoint - a.getOrigin();
    Vector3f unitImpluseTorque = positionRel.cross(-penatration.normalized());
    Vector3f angularVelPer= a.getInertiaInv() * unitImpluseTorque;
    Vector3f velocityDeltaPer = linearVelPer + angularVelPer.cross(positionRel);

    // desire velocity change
    Vector3f velocityRelBefore = a.getVelocity() + b.getVelocity();
    float resitution = 0.4f;
    Vector3f velocityRelAfter = -resitution * velocityRelBefore;
    Vector3f velocityDeltaDesire = velocityRelAfter - velocityRelBefore;

    // impluse
    Vector3f impluse = velocityDeltaDesire.cwiseQuotient(velocityDeltaPer);
    Vector3f impluseTorque = positionRel.cross(impluse);

    // apply impluse
    Vector3f velocity = a.getVelocity() + a.getMassInv() * impluse;
    a.setVelocity(velocity);
    Vector3f omega = a.getOmega() + a.getInertiaInv() * impluseTorque;
    a.setOmega(omega);

    // resolving interpenetration
    Vector3f angularInertiaWorld = positionRel.cross(contactNormal);
    angularInertiaWorld = a.getInertiaInv() * angularInertiaWorld;
    angularInertiaWorld = angularInertiaWorld.cross(positionRel);
    float angularInertia = angularInertiaWorld.dot(contactNormal);
    float linearInertia = a.getMassInv();
    float totalInertia = linearInertia + angularInertia;
    float linearMove = penatration.norm() * linearInertia / totalInertia;
    float angularMove = penatration.norm() * angularInertia / totalInertia;

    // Check that the angular move is within limits.
    float angularLimitConstant = 0.2f;
    float limit = angularLimitConstant * positionRel.norm();
    if (abs(angularMove) > limit)
    {
        float totalMove = linearMove + angularMove;
        // Set the new angular move, with the same sign as before.
        if (angularMove >= 0)
        {
            angularMove = limit;
        }
        else
        {
            angularMove = -limit;
        }
        // Make the linear move take the extra slack.
        linearMove = totalMove - angularMove;
    }

    // applying movement
    Vector3f origin = a.getOrigin() + 10*contactNormal * linearMove;
    a.setOrigin(origin);
    Matrix3f inverseInertiaTensor = a.getInertiaInv();
    Vector3f impulsiveTorque = positionRel.cross(contactNormal);
    Vector3f impulsePerMove = inverseInertiaTensor * impluseTorque;
    Vector3f rotationPerMove = impulsePerMove / angularInertia;
    Vector3f rotation = rotationPerMove * angularMove;
    Matrix3f rotationMatrix;
    rotationMatrix = AngleAxisf(rotation[0], Vector3f::UnitX())
                        * AngleAxisf(rotation[1],  Vector3f::UnitY())
                        * AngleAxisf(rotation[2], Vector3f::UnitZ());
    Matrix3f rotationChange = a.getRotation() * rotationMatrix;
    a.setRotation(rotationChange);
}
}