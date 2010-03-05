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

// STLLoader.cpp

#include <iostream>
#include <string.h>
#include <ctype.h>
#include <stdio.h>

#include "STLLoader.h"

using namespace collada_urdf;

Vector3::Vector3(float _x, float _y, float _z) : x(_x), y(_y), z(_z) { }

bool Vector3::operator==(const Vector3& v) const {
    return x == v.x && y == v.y && z == v.z;
}

Mesh::Mesh() {
}

int Mesh::getVertexIndex(const Vector3& v) const {
    for (unsigned int i = 0; i < vertices.size(); i++)
        if (vertices[i] == v)
            return i;

    return -1;
}

void Mesh::addVertex(const Vector3& v) { vertices.push_back(v); }
void Mesh::addNormal(const Vector3& n) { normals.push_back(n);  }
void Mesh::addIndex(unsigned int i)    { indices.push_back(i);  }

//

Mesh* STLLoader::load(const std::string& filename) {
    Mesh* mesh = new Mesh();

    FILE* file = fopen(filename.c_str(), "r");
    readBinary(file, mesh);
    fclose(file);

    return mesh;
}

void STLLoader::readBinary(FILE* filein, Mesh* mesh) {
    // 80 byte Header
    for (int i = 0; i < 80; i++) 
        fgetc(filein);

    int face_num = readLongInt(filein);

    for (int iface = 0; iface < face_num; iface++) {
        Vector3 normal(readFloat(filein), readFloat(filein), readFloat(filein));
        for (int i = 0; i < 3; i++) {
            Vector3 vertex(readFloat(filein), readFloat(filein), readFloat(filein));
            int index = mesh->getVertexIndex(vertex);
            if (index == -1) {
                mesh->addVertex(vertex);
                mesh->addNormal(normal);
                index = mesh->vertices.size() - 1;
            }
            mesh->addIndex(index);
        }
        readShortInt(filein);  // 2 byte attribute
    }
}

float STLLoader::readFloat(FILE* filein) {
    float rval;
    if (fread(&rval, sizeof(float), 1, filein) == 0)
        std::cerr << "Error in STLLoader::readFloat" << std::endl;
    return rval;
}

uint32_t STLLoader::readLongInt(FILE* filein) {
    union
    {
        uint32_t yint;
        char ychar[4];
    } y;
    y.ychar[0] = fgetc(filein);
    y.ychar[1] = fgetc(filein);
    y.ychar[2] = fgetc(filein);
    y.ychar[3] = fgetc(filein);

    return y.yint;
}

uint16_t STLLoader::readShortInt(FILE* filein) {
    uint8_t c1 = fgetc(filein);
    uint8_t c2 = fgetc(filein);

    uint16_t ival = c1 | (c2 << 8);

    return ival;
}
