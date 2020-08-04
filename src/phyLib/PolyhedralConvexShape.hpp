#pragma once

#include <Eigen/Eigen>
#include "ConvexShape.hpp"

///interface class for polyhedral convex shape
class PolyhedralConvexShape : public ConvexShape
{
public:
    PolyhedralConvexShape(Eigen::Vector3f origin={0.f, 0.f, 0.f},
        Eigen::Matrix3f rot=Eigen::Vector3f{1.f, 1.f, 1.f}.asDiagonal());
    virtual ~PolyhedralConvexShape(){};

    Eigen::Vector3f localGetSupportingVertex(Eigen::Vector3f dir) const;
};