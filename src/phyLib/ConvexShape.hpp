#pragma once

#include <Eigen/Eigen>
#include "CollisionShape.hpp"

///abstract interface class for convex shape
class ConvexShape : public CollisionShape
{
public:
    ConvexShape(Eigen::Vector3f origin={0.f, 0.f, 0.f},
        Eigen::Matrix3f rot=Eigen::Vector3f{1.f, 1.f, 1.f}.asDiagonal());
    virtual ~ConvexShape(){};
};