#pragma once

#include <string>
#include <vector>
#include <array>

namespace BasicGL
{
    /** 
 * Struct to store the position and normal indices.
 */
    struct MeshFaceIndices
    {
        int posIndices[3];
        int texIndices[3];
        int normalIndices[3];
    };

    /**
 * Read for OBJ files.
 */
    class Loader
    {
    private:
        /* data */
    public:
        using Vec2f = std::array<float, 2>;
        using Vec3f = std::array<float, 3>;

        /**
     * This function loads an OBJ file.
     * Only triangulated meshes are supported.
	 */
        static void loadObj(const std::string &filename, std::vector<Vec3f> *x,
                            std::vector<MeshFaceIndices> *faces, std::vector<Vec3f> *normals,
                            std::vector<Vec2f> *texcoords, const Vec3f &scale);
    };
}; // namespace BasicGL