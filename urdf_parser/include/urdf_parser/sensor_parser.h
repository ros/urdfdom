/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2016, Bielefeld University
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
*   * Neither the name of Bielefeld University nor the names of its
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

/* Author: Robert Haschke */

#ifndef URDF_PARSER_URDF_SENSOR_PARSER_H
#define URDF_PARSER_URDF_SENSOR_PARSER_H

#include <string>
#include <map>
#include <tinyxml.h>
#include <urdf_sensor/types.h>

#include "exportdecl.h"

namespace urdf {

  /// API for any custom SensorParser
  class URDFDOM_DLLAPI SensorParser {
  public:
    virtual ~SensorParser() = default;
    virtual SensorBaseSharedPtr parse(TiXmlElement &sensor_element) = 0;
  };
  URDF_TYPEDEF_CLASS_POINTER(SensorParser);

  /// map from sensor name to Sensor instance
  using SensorMap = std::map<std::string, SensorSharedPtr>;
  /// map from sensor type to SensorParser instance
  using SensorParserMap = std::map<std::string, SensorParserSharedPtr>;

  /** parse <sensor> tags in URDF document for which a parser exists in SensorParserMap */
  URDFDOM_DLLAPI SensorMap parseSensors(TiXmlDocument &urdf,  const SensorParserMap &parsers);

  /** convienency function to fetch a sensor with given name and type from the map */
  template <typename T>
  URDFDOM_DLLAPI std::shared_ptr<T> getSensor(const std::string &name, const SensorMap &sensors) {
    SensorMap::const_iterator s = sensors.find(name);
    if (s == sensors.end()) return std::shared_ptr<T>();
    else return std::dynamic_pointer_cast<T>(s->second->sensor_);
  }

}

#endif
