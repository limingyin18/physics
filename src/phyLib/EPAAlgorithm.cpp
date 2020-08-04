#include "ConvexCollision.hpp"
#include <iostream>

using namespace std;
using namespace Eigen;

constexpr float EPSILON = 1e-6f;

/* an triangle info for EPA algorithm */
struct Entry
{
    array<Vector3f, 3> y;  // the vertices of the triangle
    bool affineDependent;  // whether affinely dependent
    Vector3f v;            // the point closet to the origin on the triangle
    bool within;           // whether v within the triangle
    float dist;            // the distance of v to origin
    array<Entry *, 3> adj; // pointers to the triangle adjacent to edge i
    array<unsigned, 3> j;  /*  for each adjoining triangle $entry.adj[i]$, 
                              the index of the adjoining edge, 
                              such that $entry.adj[i].adj[entry.j] = entry$, 
                              all triangle oriented in the same direction
                           */
    bool obsolete;         // whether the triangle is visble from the new support point

    Entry() = default;
    Entry(const Entry &rhs) = delete;
    Entry(Entry &&rhs)
    {
        y = rhs.y;
        affineDependent = rhs.affineDependent;
        v = rhs.v;
        within = rhs.within;
        dist = rhs.dist;
        adj = rhs.adj;
        for (unsigned i = 0; i < 3; ++i)
            rhs.adj[i] = nullptr;
        j = rhs.j;
        obsolete = rhs.obsolete;

        for (unsigned i = 0; i < 3; ++i)
            if (adj[i] != nullptr)
                adj[i]->adj[j[i]] = this;
    }

    Entry &operator=(const Entry &rhs) = delete;

    Entry &operator=(Entry &&rhs)
    {
        y = rhs.y;
        affineDependent = rhs.affineDependent;
        v = rhs.v;
        within = rhs.within;
        dist = rhs.dist;
        adj = rhs.adj;
        for (unsigned i = 0; i < 3; ++i)
            rhs.adj[i] = nullptr;
        j = rhs.j;
        obsolete = rhs.obsolete;

        for (unsigned i = 0; i < 3; ++i)
            if (adj[i] != nullptr)
                adj[i]->adj[j[i]] = this;

        return *this;
    }

    Entry(const Vector3f &a, const Vector3f &b, const Vector3f &c)
    {
        y = {a, b, c};
        affineDependent = (b - a).cross(c - a).norm() < EPSILON;
        Vector3f O{0.f, 0.f, 0.f};
        v = pointToPlane(O, a, b, c);
        within = pointInTriangle(a, b, c, v);
        dist = v.norm();
        obsolete = false;
        for (auto &v : adj)
            v = nullptr;
    }

    // set adjacent
    void bind(unsigned ind, Entry &adjEntry, unsigned adjJ)
    {
        adj[ind] = &adjEntry;
        j[ind] = adjJ;
        adjEntry.adj[adjJ] = this;
        adjEntry.j[adjJ] = ind;
    }

    bool operator<(const Entry &rhs) const { return dist < rhs.dist; }

    struct cmp
    {
        bool operator()(Entry *a, Entry *b)
        {
            return *b < *a;
        }
    };
};

/**
 * Recursive flood-fill algorithm for retrieving the sillhouette as seen form w.
 * 
 * @param entry triangle face
 * @param i index
 * @param w new point
 * @param E set of triangle faces to be added
 * 
 * @return
 */
void silhouette(Entry &entry, unsigned i, Vector3f &w, vector<pair<Entry *, unsigned>> &E)
{
    if (!entry.obsolete) // face visited first time
    {
        if (entry.v.dot(w) < entry.v.squaredNorm()) // face is not visible from w
        {
            E.push_back({&entry, i});
        }
        else // mark entry visible, and search its neighbors
        {
            entry.obsolete = true;
            silhouette(*entry.adj[(i + 1) % 3], entry.j[(i + 1) % 3], w, E);
            silhouette(*entry.adj[(i + 2) % 3], entry.j[(i + 2) % 3], w, E);
        }
    }
}

std::optional<Eigen::Vector3f>
EPAAlgorithm(const PolyhedralConvexShape &shape1, const PolyhedralConvexShape &shape2,
             vector<Vector3f> &simplex)
{
    // add points to simplex to 4
    switch (simplex.size())
    {
    case 1:
        break;
    case 2:
        break;
    case 3:
    {
        vector<Vector3f> temp{simplex};
        Vector3f normalToTemp = (simplex[0] - simplex[1]).cross(simplex[0] - simplex[2]);
        Vector3f newX = MinkowskiDifferenceSupport(shape1, shape2, normalToTemp);
        Vector3f newY = MinkowskiDifferenceSupport(shape1, shape2, -normalToTemp);

        vector<Vector3f> newSimplex1{simplex[0], simplex[1], newX, newY};
        vector<Vector3f> newSimplex2{simplex[0], simplex[2], newX, newY};
        vector<Vector3f> newSimplex3{simplex[1], simplex[2], newX, newY};
        if (originInTetrahedron(newSimplex1))
        {
            simplex = newSimplex1;
            break;
        }
        else if (originInTetrahedron(newSimplex2))
        {
            simplex = newSimplex2;
            break;
        }
        else if (originInTetrahedron(newSimplex3))
        {
            simplex = newSimplex3;
            break;
        }
    }
    default:
        break;
    }

    // orient
    Matrix3f testOrient;
    testOrient.block<3, 1>(0, 0) = simplex[0] - simplex[3];
    testOrient.block<3, 1>(0, 1) = simplex[1] - simplex[3];
    testOrient.block<3, 1>(0, 2) = simplex[2] - simplex[3];
    if (testOrient.determinant() < 0)
    {
        std::swap(simplex[0], simplex[1]);
    }

    // convert simplex to 4 triangles, right hand point into the polyhedron
    vector<Entry> P{};
    P.reserve(1000);
    P.emplace_back(simplex[0], simplex[1], simplex[2]);
    P.emplace_back(simplex[1], simplex[0], simplex[3]);
    P.emplace_back(simplex[2], simplex[1], simplex[3]);
    P.emplace_back(simplex[0], simplex[2], simplex[3]);

    // set adjacent
    P[0].bind(0, P[1], 0);
    P[0].bind(1, P[2], 0);
    P[0].bind(2, P[3], 0);
    P[1].bind(1, P[3], 2);
    P[1].bind(2, P[2], 1);
    P[2].bind(2, P[3], 1);

    // push to priority queue
    priority_queue<Entry *, vector<Entry *>, Entry::cmp> Q{};
    for_each(P.begin(), P.end(), [&Q](auto &val) { Q.push(&val); });

    Entry *entry;
    bool closeEnough = false;
    // upper bound for the squared penetration depth
    float u = numeric_limits<float>::infinity();
    while (!closeEnough && !Q.empty() && Q.top()->v.squaredNorm() <= u)
    {
        entry = Q.top();
        Q.pop();
        if (!entry->obsolete) // facet 'entry' is a proper best candidate
        {
            Vector3f w = MinkowskiDifferenceSupport(shape1, shape2, entry->v);
            u = min(u, powf(entry->v.dot(w), 2) / entry->v.squaredNorm());
            closeEnough = u <= powf(1 + EPSILON, 2) * entry->v.squaredNorm();
            if (!closeEnough) // blow up the current polytope by adding vertex w
            {
                entry->obsolete = true; // facet 'entry' is visible from w
                vector<std::pair<Entry *, unsigned>> E{};
                for (unsigned i = 0; i < 3; ++i)
                    silhouette(*entry->adj[i], entry->j[i], w, E);

                size_t indFirst = P.size();
                for (auto &val : E) // construct new entry
                {
                    Entry *e;
                    unsigned i;
                    std::tie(e, i) = val;
                    P.emplace_back(e->y[(i + 1) % 3], e->y[i], w);
                    P.back().bind(0, *e, i);
                }

                for (unsigned i = 0; i < E.size(); ++i) // bind each other
                    P[indFirst + i].bind(1, P[indFirst + ((i + 1) % E.size())], 2);

                for (unsigned i = 0; i < E.size(); ++i)
                {
                    if (P[indFirst + i].affineDependent)
                        return entry->v;

                    if (P[indFirst + i].within &&
                        entry->v.squaredNorm() <= P[indFirst + i].v.squaredNorm() &&
                        P[indFirst + i].v.squaredNorm() <= u)
                        Q.push(&P[indFirst + i]);
                }
            }
        }
    }
    return entry->v;
}
