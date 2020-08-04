#include "PolyhedralConvexShape.hpp"

using namespace Eigen;
using namespace std;

constexpr float EPSILON = 1e-6f;

PolyhedralConvexShape::PolyhedralConvexShape(Vector3f origin, Matrix3f rot) : 
	ConvexShape(origin, rot)
{

}

Vector3f PolyhedralConvexShape::localGetSupportingVertex(Vector3f dir) const
{
	if (dir.norm() < EPSILON)
	{
		dir = Vector3f{1.f, 0.f, 0.f};
	}
	else
	{
       dir.normalize();
	}

    Vector3f supVertex(0.f, 0.f, 0.f);
	float tempDot = -numeric_limits<float>::max();
	Matrix4f trans = getTransform();
	for (size_t i = 0; i < getNumVertices(); i++)
	{
        Vector4f tempVertexH = {1.0f, 1.0f, 1.0f, 1.0f};
		Vector3f tempVertex = getVertex(i);
		tempVertexH[0] = tempVertex[0];
		tempVertexH[1] = tempVertex[1];
		tempVertexH[2] = tempVertex[2];
		tempVertexH = trans * tempVertexH;
		Vector3f vertex = {tempVertexH[0] / tempVertexH[3], 
						   tempVertexH[1] / tempVertexH[3],
						   tempVertexH[2] / tempVertexH[3]};
        float newDot = vertex.dot(dir);
		if (newDot > tempDot)
		{
			tempDot = newDot;
			supVertex = vertex;
		}
	}

	return supVertex;
}