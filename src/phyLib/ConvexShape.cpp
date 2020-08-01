#include "ConvexShape.hpp"

using namespace Eigen;

ConvexShape::ConvexShape(Vector3f origin, Matrix3f rot) : CollisionShape(origin, rot)
{

}