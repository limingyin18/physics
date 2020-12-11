#pragma once

#include <array>
#include <vector>
#include <limits>
#include <Eigen/Eigen>

class AABBTree
{
    struct Node
    {
        Node() : mNumFaces(0), mFaces(nullptr), mMinExtents(0.f, 0.f, 0.f), mMaxExtents(0.f, 0.f, 0.f)
        {
        }

        union
        {
            unsigned mChildren;
            unsigned mNumFaces;
        };

        unsigned *mFaces;
        Eigen::Vector3f mMinExtents;
        Eigen::Vector3f mMaxExtents;
    };

    struct Bounds
    {
        Bounds() : mMin(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max()),
                   mMax(-std::numeric_limits<float>::max(), -std::numeric_limits<float>::max(), -std::numeric_limits<float>::max())
        {
        }

        Bounds(const Bounds& rhs) : mMin(rhs.mMin), mMax(rhs.mMax)
        {
        }

        Bounds(const Eigen::Vector3f &min, const Eigen::Vector3f &max) : 
            mMin(min), mMax(max)
        {
        }

        float GetVolume() const
        {
            Eigen::Vector3f e = mMax - mMin;
            return e.prod();
        }

        float GetSurfaceArea() const
        {
            Eigen::Vector3f e = mMax-mMin;
            return 2.0f*(e[0]*e[1] + e[0]*e[2] + e[1]*e[2]);
        }

        void Union(const Bounds &b)
        {
            mMin[0] = std::min(mMin[0], b.mMin[0]);
            mMin[1] = std::min(mMin[1], b.mMin[1]);
            mMin[2] = std::min(mMin[2], b.mMin[2]);

            mMax[0] = std::max(mMax[0], b.mMax[0]);
            mMax[1] = std::max(mMax[1], b.mMax[1]);
            mMax[2] = std::max(mMax[2], b.mMax[2]);
        }

        Eigen::Vector3f mMin;
        Eigen::Vector3f mMax;
    };

    struct FaceSorter
    {
        FaceSorter(const std::array<float, 3> *positions, const unsigned *indices, unsigned n, unsigned axis)
            : mVertices(positions), mIndices(indices), mNumIndices(n), mAxis(axis)
        {}

        bool operator()(unsigned lhs, unsigned rhs) const
        {
            float a = GetCentroid(lhs);
            float b = GetCentroid(rhs);

            if(a == b)
                return lhs < rhs;
            else
                return a < b;
        }

        float GetCentroid(unsigned face) const
        {
            const std::array<float, 3> &a = mVertices[mIndices[face*3+0]];
            const std::array<float, 3> &b = mVertices[mIndices[face*3+1]];
            const std::array<float, 3> &c = mVertices[mIndices[face*3+2]];

            return (a[mAxis] + b[mAxis] + c[mAxis])/ 3.0f;
        }

        const std::array<float, 3> *mVertices;
        const unsigned *mIndices;
        unsigned mNumIndices;
        unsigned mAxis;
    };

private:
    const std::array<float, 3> *mVertices = nullptr;
    unsigned mNumVerts = 0;
    const unsigned *mIndices = nullptr;
    unsigned mNumFaces = 0;

    std::vector<unsigned> mFaces;
    std::vector<Node> mNodes;
    std::vector<Bounds> mFaceBounds;

    // track the next free node
    unsigned mFreeNode;

    // stats
    unsigned mTreeDepth = 0;
    unsigned mInnerNodes = 0;
    unsigned mLeafNodes = 0;

    unsigned GetNumFaces() const { return mNumFaces; }
	unsigned GetNumNodes() const { return mNodes.size(); }

    void CalculateFaceBounds(unsigned *faces, unsigned numFaces,
        Eigen::Vector3f& outMinExtents, Eigen::Vector3f& outMaxExtents);

    void BuildRecursive(unsigned nodeIndex, unsigned *faces, unsigned numFaces);
    unsigned PartitionSAH(Node& n, unsigned *faces, unsigned numFaces);

public:
    AABBTree(const std::array<float, 3> *vertices, unsigned numVerts,
             const unsigned *indices, unsigned numFaces);
    ~AABBTree();

};