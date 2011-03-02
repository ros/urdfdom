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

/* Author: Wim Meeussen */

#include <boost/algorithm/string.hpp>
#include <ros/ros.h>
#include <vector>
#include "urdf/model.h"

namespace urdf{

bool urdfFromColladaFile(std::string const& daefilename, Model& model);
bool urdfFromColladaData(std::string const& data, Model& model);
bool urdfFromTiXML(TiXmlElement *robot_xml, Model& model);
bool IsColladaFile(const std::string& filename);
bool IsColladaData(const std::string& data);


bool Model::initFile(const std::string& filename)
{
  // necessary for COLLADA compatibility
  if( IsColladaFile(filename) ) {
    return urdfFromColladaFile(filename,*this);
  }
  TiXmlDocument xml_doc;
  xml_doc.LoadFile(filename);

  return init(&xml_doc);
}


bool Model::initParam(const std::string& param)
{
  ros::NodeHandle nh;
  std::string xml_string;
  
  // gets the location of the robot description on the parameter server
  std::string full_param;
  if (!nh.searchParam(param, full_param)){
    ROS_ERROR("Could not find parameter %s on parameter server", param.c_str());
    return false;
  }

  // read the robot description from the parameter server
  if (!nh.getParam(full_param, xml_string)){
    ROS_ERROR("Could not read parameter %s on parameter server", full_param.c_str());
    return false;
  }
  return initString(xml_string);
}


bool Model::initString(const std::string& xml_string)
{
  // necessary for COLLADA compatibility
  if( IsColladaData(xml_string) ) {
    return urdfFromColladaData(xml_string,*this);
  }

  TiXmlDocument xml_doc;
  xml_doc.Parse(xml_string.c_str());

  return init(&xml_doc);
}


bool Model::initXml(TiXmlDocument *xml_doc)
{
  if (!xml_doc)
  {
    ROS_ERROR("Could not parse the xml");
    return false;
  }

  // necessary for COLLADA compatibility
  if( !!xml_doc->RootElement() ) {
    if( std::string("COLLADA") == xml_doc->RootElement()->ValueStr() ) {
      return urdfFromTiXML(xml_doc->RootElement(),*this);
    }
  }

  return init(xml_doc);
}

bool Model::initXml(TiXmlElement *robot_xml)
{
  ROS_DEBUG("Parsing robot xml");
  if (!robot_xml) return false;

  // necessary for COLLADA compatibility
  if( std::string("COLLADA") == robot_xml->ValueStr() ) {
    return urdfFromTiXML(robot_xml,*this);
  }

  return init(robot_xml);
}

}// namespace
