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
*   * Redistributions of source code must retain the above copyright
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

/* Author: Wim Meeussen, John Hsu */


#include <urdf_model/pose.h>
#include <fstream>
#include <sstream>
#include <boost/lexical_cast.hpp>
#include <algorithm>
#include <urdf_parser/exceptions.h>

namespace urdf{

void Vector3::init(const std::string &vector_str)
{
  this->clear();
  std::vector<std::string> pieces;
  std::vector<double> xyz;
  boost::split( pieces, vector_str, boost::is_any_of(" "));
  for (unsigned int i = 0; i < pieces.size(); ++i){
    if (pieces[i] != ""){
      try {
        xyz.push_back(boost::lexical_cast<double>(pieces[i].c_str()));
      }
      catch (boost::bad_lexical_cast &e) {
        throw ParseError("Vector3 xyz element ("+ pieces[i] +") is not a valid float");
      }
    }
  }

  if (xyz.size() != 3) {
    std::stringstream stm;
    stm << "Vector contains " << xyz.size()  << "elements instead of 3 elements";
    throw ParseError(stm.str());
  }

  this->x = xyz[0];
  this->y = xyz[1];
  this->z = xyz[2];

};

void Rotation::init(const std::string &rotation_str)
{
  this->clear();

  Vector3 rpy;
  
  try {
    rpy.init(rotation_str);
  }
  catch (ParseError &e) {
    throw e.addMessage("malfomed rpy string ["+rotation_str+"]");
  }
    
};

void Pose::initXml(TiXmlElement* xml)
{
  this->clear();
  if (xml)
  {
    const char* xyz_str = xml->Attribute("xyz");
    if (xyz_str != NULL)
    {
      try {
        this->position.init(xyz_str);
      }
      catch (ParseError &e) {
        throw e.addMessage("malformed xyz string ["+std::string(xyz_str)+"]");
      }
    }

    const char* rpy_str = xml->Attribute("rpy");
    if (rpy_str != NULL)
    {
      try {
        this->rotation.init(rpy_str);
      }
      catch (ParseError &e) {
        this->rotation.clear();
        throw e.addMessage("malformed rpy ["+std::string(rpy_str)+"]");
      }
    }

  }
};

}


