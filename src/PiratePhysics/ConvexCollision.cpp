#include "ConvexCollision.hpp"

using namespace std;
using namespace Eigen;
using namespace PiratePhysics;

constexpr float EPSILON = 1e-6f;

bool originInTetrahedron(vector<Vector3f> &s)
{
    Vector3f A = s[3];
    Vector3f B = s[2];
    Vector3f C = s[1];
    Vector3f D = s[0];

    Vector3f AO =   - A;
    Vector3f AB = B - A;
    Vector3f AC = C - A;
    Vector3f AD = D - A;

    Vector3f ABC = AB.cross(AC); // normal to ABC
    Vector3f ACD = AC.cross(AD); // normal to ACD
    Vector3f ADB = AD.cross(AB); // normal to ADB

    float AD_ABC = ABC.dot(AD); // AD project on normal of ABC
    float AB_ACD = ACD.dot(AB); // AB project on normal of ACD
    float AC_ADB = ADB.dot(AC); // AC project on normal of ADB

    float AO_ABC = ABC.dot(AO); // AO project on normal of ABC
    float AO_ACD = ACD.dot(AO); // AO project on normal of ACD
    float AO_ADB = ADB.dot(AO); // AO project on normal of ADB

    bool inside_ABC = AD_ABC * AO_ABC > EPSILON;
    bool inside_ACD = AB_ACD * AO_ACD > EPSILON;
    bool inside_ADB = AC_ADB * AO_ADB > EPSILON;

    if(inside_ABC && inside_ACD && inside_ADB) // origin inside tetrahedron
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool pointInTriangle(const Eigen::Vector3f &A, const Eigen::Vector3f &B,
    const Eigen::Vector3f &C, const Eigen::Vector3f &P)
{    
    Vector3f v0 = C - A;
    Vector3f v1 = B - A;
    Vector3f v2 = P - A;
    float dot00 = v0.dot(v0);
    float dot01 = v0.dot(v1);
    float dot02 = v0.dot(v2);
    float dot11 = v1.dot(v1);
    float dot12 = v1.dot(v2);

    Matrix2f ACoff;      
    ACoff<< dot00, dot01, 
            dot01, dot11;          
    Vector2f b(dot02, dot12);   
    Vector2f x = ACoff.lu().solve(b);

    if (x[0] < 0 || x[0] > 1) // u out of range
    {        
        return false;
    }

    if (x[1] < 0 || x[1] > 1) // v out of range
    {        
        return false;
    }
    
    return x[0] + x[1] <= 1;
}

Vector3f pointToPlane(const Vector3f &p, const Vector3f &p1, const Vector3f &n)
{
    float t = (n.dot(p1) - n.dot(p)) / n.squaredNorm();
    return p + n*t;
}

Vector3f pointToPlane(const Vector3f &p, 
    const Vector3f &p1, const Vector3f &p2, const Vector3f &p3)
{
    Vector3f p2p1 = p2 - p1;
    Vector3f p3p1 = p3 - p1;

    Vector3f n = p2p1.cross(p3p1);

    // turn to normal method
    return pointToPlane(p, p1, n);
}

Vector3f
MinkowskiDifferenceSupport(const CollisionShape &shape1, 
    const CollisionShape &shape2, const Vector3f &dir)
{
    Vector3f a = shape1.localGetSupportingVertex(dir);
    Vector3f b = shape2.localGetSupportingVertex(-dir);
    return a - b;
}

std::optional<Vector3f>
collisionDetection(const CollisionShape &shape1, 
    const CollisionShape &shape2)
{
    auto simplex = GJKAlgorithm(shape1, shape2);
    if(simplex)
        return EPAAlgorithm(shape1, shape2, simplex.value());
    else
        return std::nullopt;
}