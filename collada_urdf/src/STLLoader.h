/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redstributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Tim Field */

// STLLoader.h

#ifndef COLLADA_URDF_STLLOADER_HH
#define COLLADA_URDF_STLLOADER_HH

#include <stdint.h>
#include <string>
#include <vector>

#define LINE_MAX_LEN 256
#define COR3_MAX 200000
#define ORDER_MAX 10
#define FACE_MAX 200000

namespace collada_urdf
{
    class Vector3
    {
    public:
        Vector3(float x, float y, float z);

        bool operator==(const Vector3& v) const;

        float x;
        float y;
        float z;
    };

    class Mesh
    {
    public:
        Mesh();

        bool         hasVertex(const Vector3& v) const;
        unsigned int getVertexIndex(const Vector3& v) const;

        void addVertex(const Vector3& v);
        void addNormal(const Vector3& n);
        void addIndex(unsigned int i);

    private:
        std::vector<Vector3>      vertices;
        std::vector<Vector3>      normals;
        std::vector<unsigned int> indices;
    };

    class STLLoader
    {
    public:
        Mesh* load(const std::string& filename);

    private:
        void     readBinary(FILE* filein, Mesh* mesh);
        uint32_t readLongInt(FILE* filein);
        uint16_t readShortInt(FILE* filein);
        float    readFloat(FILE* filein);
    };
}

#endif
