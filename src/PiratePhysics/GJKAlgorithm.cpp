#include "ConvexCollision.hpp"

using namespace std;
using namespace Eigen;
using namespace PiratePhysics;

namespace PiratePhysics
{
constexpr float EPSILON = 1e-6f;

bool Simplex2(vector<Vector3f> &s, Vector3f &d)
{
    Vector3f A = s[1];
    Vector3f B = s[0];
    Vector3f AB = B - A;
    Vector3f AO = -A;
    Vector3f ABOO = AB.cross(AO).cross(AB); // norm to AB toward origin

    if (ABOO.norm() <= EPSILON) // origin lies on
    {
        return true;
    }
    else
    {
        d = ABOO;
        return false;
    }
}

bool Simplex3(vector<Vector3f> &s, Vector3f &d)
{
    Vector3f A = s[2];
    Vector3f B = s[1];
    Vector3f C = s[0];
    Vector3f AB = B - A;
    Vector3f AC = C - A;
    Vector3f AO = -A;
    Vector3f ABC = AB.cross(AC);
    Vector3f ACBB = AC.cross(AB).cross(AB); // norm to AB toward far from C
    Vector3f ABCC = AB.cross(AC).cross(AC); // norm to AC toward far from B

    if (ACBB.dot(AO) > EPSILON) // origin lies on outside of AB
    {
        d = ACBB;
        return false;
    }

    if (ABCC.dot(AO) > EPSILON) // origin lies on outside of AC
    {
        d = ABCC;
        return false;
    }

    float dot = ABC.dot(AO); // origin lies in ABC region
    if (dot > EPSILON)
    {
        d = ABC;
        return false;
    }
    else if (dot < -EPSILON)
    {
        d = -ABC;
        return false;
    }
    else // origin lies in ABC triangle
    {
        return true;
    }
}

bool Simplex4(vector<Vector3f> &s, Vector3f &d)
{
    Vector3f A = s[3];
    Vector3f B = s[2];
    Vector3f C = s[1];
    Vector3f D = s[0];

    Vector3f AO = -A;
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

    if (inside_ABC && inside_ACD && inside_ADB) // origin inside tetrahedron
    {
        return true;
    }
    else if (!inside_ABC) // origin outside ABC
    {
        // remove D
        s[0] = s[1];
        s[1] = s[2];
        s[2] = s[3];
        s.pop_back();
    }
    else if (!inside_ACD) // origin outside ACD
    {
        // remove B
        s[2] = s[3];
        s.pop_back();
    }
    else // origin outside ADB
    {
        // remove C
        s[1] = s[2];
        s[2] = s[3];
        s.pop_back();
    }

    return Simplex3(s, d);
}

bool SimplexOrigin(vector<Vector3f> &s, Vector3f &d)
{
    bool contain_origin = false;
    switch (s.size())
    {
    case 2: // line segment
        contain_origin = Simplex2(s, d);
        break;
    case 3: // triangle
        contain_origin = Simplex3(s, d);
        break;
    case 4: // tetrahedron
        contain_origin = Simplex4(s, d);
        break;
    default:
        break;
    }

    return contain_origin;
}

std::optional<vector<Vector3f>>
GJKAlgorithm(const CollisionShape &shape1, const CollisionShape &shape2)
{
    // center difference as initial direction
    Vector3f dir = shape1.getOrigin() - shape2.getOrigin();
    if (dir.norm() < 0.001f)
        dir = Vector3f{1.0f, 0.f, 0.f};
    dir.normalize();

    // init
    vector<Vector3f> simplex;
    simplex.reserve(4);

    Vector3f simplex_point = MinkowskiDifferenceSupport(shape1, shape2, dir);
    simplex.push_back(simplex_point);
    dir = -simplex_point;

    // main loop
    int max_iteration = max(shape1.getNumVertices(), shape2.getNumVertices());
    while (max_iteration-- > 0)
    {
        simplex_point = MinkowskiDifferenceSupport(shape1, shape2, dir);

        if (simplex_point.dot(dir) < 0)
            return std::nullopt;

        simplex.push_back(simplex_point);

        if (SimplexOrigin(simplex, dir))
            return simplex;
    }

    return std::nullopt;
}
}