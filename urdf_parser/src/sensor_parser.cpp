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

/* Author: John Hsu */


#include "urdf_parser/sensor_parser.h"
#include "urdf_parser/pose.h"
#include <urdf_sensor/camera.h>
#include <urdf_sensor/ray.h>

#include <boost/lexical_cast.hpp>
#include <console_bridge/console.h>

namespace urdf {

SensorBaseSharedPtr parseSensorBase(TiXmlElement *sensor_xml, const SensorParserMap &parsers)
{
  // find first child element that is not <parent> or <origin>
  const char* sensor_type;
  TiXmlElement *sensor_base_xml = sensor_xml->FirstChildElement();
  while (sensor_base_xml) {
    sensor_type = sensor_base_xml->Value();
    if (strcmp(sensor_type, "parent") && strcmp(sensor_type, "origin"))
      break;
    sensor_base_xml = sensor_base_xml->NextSiblingElement();
  }

  if (sensor_base_xml)
  {
    SensorParserMap::const_iterator parser = parsers.find(sensor_type);
    if (parser != parsers.end() && parser->second)
    {
      return parser->second->parse(*sensor_base_xml);
    }
    else
    {
      CONSOLE_BRIDGE_logDebug("Sensor type not handled: %s", sensor_type);
    }
  }
  else
  {
    CONSOLE_BRIDGE_logError("No child element defining the sensor.");
  }

  return SensorBaseSharedPtr();
}


bool parseSensor(Sensor &sensor, TiXmlElement* config, const SensorParserMap &parsers)
{
  sensor.clear();

  const char *name_char = config->Attribute("name");
  if (!name_char)
  {
    CONSOLE_BRIDGE_logError("No name given for the sensor.");
    return false;
  }
  sensor.name_ = std::string(name_char);

  // parse parent link name
  TiXmlElement *parent_xml = config->FirstChildElement("parent");
  const char *parent_link_name_char = parent_xml ? parent_xml->Attribute("link") : NULL;
  if (!parent_link_name_char)
  {
    CONSOLE_BRIDGE_logError("No parent link name given for the sensor.");
    return false;
  }
  sensor.parent_link_ = std::string(parent_link_name_char);

  // parse origin
  TiXmlElement *o = config->FirstChildElement("origin");
  if (o)
  {
    if (!parsePose(sensor.origin_, o))
      return false;
  }

  // parse sensor
  sensor.sensor_ = parseSensorBase(config, parsers);
  return static_cast<bool>(sensor.sensor_);
}

URDFDOM_DLLAPI
SensorMap parseSensors(TiXmlDocument &urdf_xml,  const SensorParserMap &parsers)
{
  TiXmlElement *robot_xml = urdf_xml.FirstChildElement("robot");
  if (!robot_xml) {
    CONSOLE_BRIDGE_logError("Could not find the 'robot' element in the URDF");
  }

  SensorMap results;
  // Get all sensor elements
  for (TiXmlElement* sensor_xml = robot_xml->FirstChildElement("sensor");
       sensor_xml; sensor_xml = sensor_xml->NextSiblingElement("sensor"))
  {
    SensorSharedPtr sensor;
    sensor.reset(new Sensor);

    if (parseSensor(*sensor, sensor_xml, parsers))
    {
      if (results.find(sensor->name_) != results.end())
      {
        CONSOLE_BRIDGE_logWarn("Sensor '%s' is not unique. Ignoring consecutive ones.", sensor->name_.c_str());
      }
      else
      {
        results.insert(make_pair(sensor->name_, sensor));
        CONSOLE_BRIDGE_logDebug("urdfdom: successfully added a new sensor '%s'", sensor->name_.c_str());
      }
    }
    else
    {
      CONSOLE_BRIDGE_logError("failed to parse sensor element");
    }
  }
  return results;
}

}
