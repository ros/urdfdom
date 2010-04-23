/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2010, Willow Garage, Inc.
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

#ifndef COLLADA_URDF_STL_LOADER_H
#define COLLADA_URDF_STL_LOADER_H

#include <stdint.h>

#include <string>
#include <vector>

#include <boost/shared_ptr.hpp>

namespace collada_urdf {

class Vector3
{
public:
    Vector3(float x, float y, float z);

    bool operator==(Vector3 const& v) const;

    float x;
    float y;
    float z;
};

class Mesh
{
public:
    Mesh();

    int getVertexIndex(Vector3 const& v) const;

    void addVertex(Vector3 const& v);
    void addNormal(Vector3 const& n);
    void addIndex(unsigned int i);

public:
    std::vector<Vector3>      vertices;
    std::vector<Vector3>      normals;
    std::vector<unsigned int> indices;
};

class STLLoader
{
public:
    boost::shared_ptr<Mesh> load(std::string const& filename);

private:
    FILE*                   file_;
    boost::shared_ptr<Mesh> mesh_;

    void     readBinary();
    uint32_t readLongInt();
    uint16_t readShortInt();
    float    readFloat();
};

}

#endif
