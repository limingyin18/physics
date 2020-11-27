#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include "Loader.hpp"

using namespace BasicGL;
using namespace std;

namespace BasicGL
{
    static void tokenize(const std::string &str, std::vector<std::string> &tokens, const std::string &delimiters = " ")
    {
        std::string::size_type lastPos = str.find_first_not_of(delimiters, 0);
        std::string::size_type pos = str.find_first_of(delimiters, lastPos);

        while (std::string::npos != pos || std::string::npos != lastPos)
        {
            tokens.push_back(str.substr(lastPos, pos - lastPos));
            lastPos = str.find_first_not_of(delimiters, pos);
            pos = str.find_first_of(delimiters, lastPos);
        }
    }

    void Loader::loadObj(const std::string &filename, std::vector<Vec3f> *x,
                                std::vector<MeshFaceIndices> *faces, std::vector<Vec3f> *normals,
                                std::vector<Vec2f> *texcoords, const Vec3f &scale)
    {
        clog << "Loading " << filename << endl;

        std::ifstream filestream;
        filestream.open(filename.c_str());
        if (filestream.fail())
        {
            cerr << "Failed to open file: " << filename << endl;
            return;
        }

        std::string line_stream;
        bool vt = false;
        bool vn = false;

        std::vector<std::string> pos_buffer;
        std::vector<std::string> f_buffer;

        while (getline(filestream, line_stream))
        {
            std::stringstream str_stream(line_stream);
            std::string type_str;
            str_stream >> type_str;

            if (type_str == "v")
            {
                Vec3f pos;
                pos_buffer.clear();
                std::string parse_str = line_stream.substr(line_stream.find("v") + 1);
                tokenize(parse_str, pos_buffer);
                for (unsigned int i = 0; i < 3; i++)
                    pos[i] = stof(pos_buffer[i]) * scale[i];

                x->push_back(pos);
            }
            else if (type_str == "vt")
            {
                if (texcoords != nullptr)
                {
                    Vec2f tex;
                    pos_buffer.clear();
                    std::string parse_str = line_stream.substr(line_stream.find("vt") + 2);
                    tokenize(parse_str, pos_buffer);
                    for (unsigned int i = 0; i < 2; i++)
                        tex[i] = stof(pos_buffer[i]);

                    texcoords->push_back(tex);
                    vt = true;
                }
            }
            else if (type_str == "vn")
            {
                if (normals != nullptr)
                {
                    Vec3f nor;
                    pos_buffer.clear();
                    std::string parse_str = line_stream.substr(line_stream.find("vn") + 2);
                    tokenize(parse_str, pos_buffer);
                    for (unsigned int i = 0; i < 3; i++)
                        nor[i] = stof(pos_buffer[i]);

                    normals->push_back(nor);
                    vn = true;
                }
            }
            else if (type_str == "f")
            {
                MeshFaceIndices faceIndex;
                if (vn && vt)
                {
                    f_buffer.clear();
                    std::string parse_str = line_stream.substr(line_stream.find("f") + 1);
                    tokenize(parse_str, f_buffer);
                    for (int i = 0; i < 3; ++i)
                    {
                        pos_buffer.clear();
                        tokenize(f_buffer[i], pos_buffer, "/");
                        faceIndex.posIndices[i] = stoi(pos_buffer[0]);
                        faceIndex.texIndices[i] = stoi(pos_buffer[1]);
                        faceIndex.normalIndices[i] = stoi(pos_buffer[2]);
                    }
                }
                else if (vn)
                {
                    f_buffer.clear();
                    std::string parse_str = line_stream.substr(line_stream.find("f") + 1);
                    tokenize(parse_str, f_buffer);
                    for (int i = 0; i < 3; ++i)
                    {
                        pos_buffer.clear();
                        tokenize(f_buffer[i], pos_buffer, "/");
                        faceIndex.posIndices[i] = stoi(pos_buffer[0]);
                        faceIndex.normalIndices[i] = stoi(pos_buffer[1]);
                    }
                }
                else if (vt)
                {
                    f_buffer.clear();
                    std::string parse_str = line_stream.substr(line_stream.find("f") + 1);
                    tokenize(parse_str, f_buffer);
                    for (int i = 0; i < 3; ++i)
                    {
                        pos_buffer.clear();
                        tokenize(f_buffer[i], pos_buffer, "/");
                        faceIndex.posIndices[i] = stoi(pos_buffer[0]);
                        faceIndex.texIndices[i] = stoi(pos_buffer[1]);
                    }
                }
                else
                {
                    f_buffer.clear();
                    std::string parse_str = line_stream.substr(line_stream.find("f") + 1);
                    tokenize(parse_str, f_buffer);
                    for (int i = 0; i < 3; ++i)
                    {
                        faceIndex.posIndices[i] = stoi(f_buffer[i]);
                    }
                }
                faces->push_back(faceIndex);
            }
        }
        filestream.close();
    }
} // namespace BasicGL