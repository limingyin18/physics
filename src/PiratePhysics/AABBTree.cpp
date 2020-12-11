#include "AABBTree.hpp"

using namespace Eigen;
using namespace std;

static unsigned sDepth = 0;

AABBTree::AABBTree(const std::array<float, 3> *vertices, unsigned numVerts,
                   const unsigned *indices, unsigned numFaces) : mVertices(vertices), mNumVerts(numVerts),
                                                                              mIndices(indices), mNumFaces(numFaces)
{
    mFaces.reserve(mNumFaces);
    mFaceBounds.reserve(mNumFaces);

    for (unsigned i = 0; i < numFaces; ++i)
    {
        Bounds top;
        CalculateFaceBounds(&i, 1, top.mMin, top.mMax);
        mFaces.push_back(i);
        mFaceBounds.push_back(top);
    }

    mNodes.reserve(static_cast<unsigned>(numFaces*1.5f));
    mFreeNode = 1;

    BuildRecursive(0, &mFaces[0], numFaces);
    assert(sDepth == 0);
}

AABBTree::~AABBTree()
{
    mIndices = nullptr;
    mVertices = nullptr;
}

void AABBTree::CalculateFaceBounds(unsigned *faces, unsigned numFaces,
                                   Eigen::Vector3f &outMinExtents, Eigen::Vector3f &outMaxExtents)
{
    Vector3f minExtents(numeric_limits<float>::max(), numeric_limits<float>::max(), numeric_limits<float>::max());
    Vector3f maxExtents(-numeric_limits<float>::max(), -numeric_limits<float>::max(), -numeric_limits<float>::max());

    // calculate face bounds
    for(unsigned i = 0; i < numFaces; ++i)
    {
        array<float, 3> a = mVertices[mIndices[faces[i]*3+0]];
        array<float, 3> b = mVertices[mIndices[faces[i]*3+1]];
        array<float, 3> c = mVertices[mIndices[faces[i]*3+2]];

        minExtents[0] = min(a[0], minExtents[0]);
        minExtents[1] = min(a[1], minExtents[1]);
        minExtents[2] = min(a[2], minExtents[2]);
        maxExtents[0] = max(a[0], maxExtents[0]);
        maxExtents[1] = max(a[1], maxExtents[1]);
        maxExtents[2] = max(a[2], maxExtents[2]);

        minExtents[0] = min(b[0], minExtents[0]);
        minExtents[1] = min(b[1], minExtents[1]);
        minExtents[2] = min(b[2], minExtents[2]);
        maxExtents[0] = max(b[0], maxExtents[0]);
        maxExtents[1] = max(b[1], maxExtents[1]);
        maxExtents[2] = max(b[2], maxExtents[2]);

        minExtents[0] = min(c[0], minExtents[0]);
        minExtents[1] = min(c[1], minExtents[1]);
        minExtents[2] = min(c[2], minExtents[2]);
        maxExtents[0] = max(c[0], maxExtents[0]);
        maxExtents[1] = max(c[1], maxExtents[1]);
        maxExtents[2] = max(c[2], maxExtents[2]);
    }

    outMinExtents = minExtents;
    outMaxExtents = maxExtents;
}

void AABBTree::BuildRecursive(unsigned nodeIndex, unsigned *faces, unsigned numFaces)
{
    const unsigned kMaxFacesPerLeaf = 6;

    // allocate more memory
    if(nodeIndex >= mNodes.size())
    {
        unsigned s = max(static_cast<unsigned>(1.5f*mNodes.size()), 512U);
        mNodes.resize(s);
    }

    Node &n = mNodes[nodeIndex];
    ++sDepth;
    mTreeDepth = max(mTreeDepth, sDepth);

    CalculateFaceBounds(faces, numFaces, n.mMinExtents, n.mMaxExtents);

    // calculate bounds of faces and add node
    if(numFaces <= kMaxFacesPerLeaf)
    {
        n.mFaces = faces;
        n.mNumFaces = numFaces;
        ++mLeafNodes;
    }
    else
    {
        ++mInnerNodes;

        // face counts for each branch
        const unsigned leftCount = PartitionSAH(n, faces, numFaces);
        const unsigned rightCount = numFaces - leftCount;

        // alloc
        mNodes[nodeIndex].mChildren = mFreeNode;
        mFreeNode += 2;

        // split faces in half and build each side recursively
        BuildRecursive(mNodes[nodeIndex].mChildren+0, faces, leftCount);
        BuildRecursive(mNodes[nodeIndex].mChildren+1, faces+leftCount, rightCount);
    }

    --sDepth;
    
}

unsigned AABBTree::PartitionSAH(Node &n, unsigned *faces, unsigned numFaces)
{
    unsigned bestAxis = 0;
    unsigned bestIndex = 0;
    float bestCost = numeric_limits<float>::max();

    for(unsigned a = 0; a < 3; ++a)
    {
        // sort faces by centroids
        		// sort faces by centroids
		FaceSorter predicate(&mVertices[0], &mIndices[0], mNumFaces*3, a);
		std::sort(faces, faces+numFaces, predicate);

        		// two passes over data to calculate upper and lower bounds
		vector<float> cumulativeLower(numFaces);
		vector<float> cumulativeUpper(numFaces);

		Bounds lower;
		Bounds upper;

		for (uint32_t i=0; i < numFaces; ++i)
		{
			lower.Union(mFaceBounds[faces[i]]);
			upper.Union(mFaceBounds[faces[numFaces-i-1]]);

			cumulativeLower[i] = lower.GetSurfaceArea();        
			cumulativeUpper[numFaces-i-1] = upper.GetSurfaceArea();
		}

		float invTotalSA = 1.0f / cumulativeUpper[0];

		// test all split positions
		for (uint32_t i=0; i < numFaces-1; ++i)
		{
			float pBelow = cumulativeLower[i] * invTotalSA;
			float pAbove = cumulativeUpper[i] * invTotalSA;

			float cost = 0.125f + (pBelow*i + pAbove*(numFaces-i));
			if (cost <= bestCost)
			{
				bestCost = cost;
				bestIndex = i;
				bestAxis = a;
			}
		}
    }

    // re-sort by best axis
	FaceSorter predicate(&mVertices[0], &mIndices[0], mNumFaces*3, bestAxis);
	std::sort(faces, faces+numFaces, predicate);

	return bestIndex+1;
}