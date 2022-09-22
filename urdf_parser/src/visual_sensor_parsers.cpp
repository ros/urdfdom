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

#include "urdf_parser/visual_sensor_parsers.h"
#include "urdf_parser/utils.h"
#include "urdf_parser/pose.h"
#include <urdf_sensor/camera.h>
#include <urdf_sensor/ray.h>

#include <console_bridge/console.h>

namespace urdf {

SensorBase* CameraParser::parse(TiXmlElement &config)
{
  TiXmlElement *image = config.FirstChildElement("image");
  if (image)
  {
    try {
      std::unique_ptr<Camera> camera(new Camera());
      camera->width = parseAttribute<unsigned int>(*image, "width");
      camera->height = parseAttribute<unsigned int>(*image, "height");
      camera->format = parseAttribute<std::string>(*image, "format");
      camera->hfov = parseAttribute<double>(*image, "hfov");
      camera->near = parseAttribute<double>(*image, "near");
      camera->far = parseAttribute<double>(*image, "far");
      return camera.release();
    }
    catch (const std::exception &e)
    {
      CONSOLE_BRIDGE_logError("Camera sensor %s", e.what());
      return nullptr;
    }
  }
  else
  {
    CONSOLE_BRIDGE_logError("Camera sensor has no <image> element");
    return nullptr;
  }
}


SensorBase* RayParser::parse(TiXmlElement &config)
{
  TiXmlElement *horizontal = config.FirstChildElement("horizontal");
  if (horizontal)
  {
    try {
      std::unique_ptr<Ray> ray(new Ray());
      ray->horizontal_samples = parseAttribute<unsigned int>(*horizontal, "samples");
      ray->horizontal_resolution = parseAttribute<double>(*horizontal, "resolution");
      ray->horizontal_min_angle = parseAttribute<double>(*horizontal, "min_angle");
      ray->horizontal_max_angle = parseAttribute<double>(*horizontal, "max_angle");
      return ray.release();
    }
    catch (const std::exception &e)
    {
      CONSOLE_BRIDGE_logError("Ray horizontal: %s", e.what());
      return nullptr;
    }
  }

  TiXmlElement *vertical = config.FirstChildElement("vertical");
  if (vertical)
  {
    try {
      std::unique_ptr<Ray> ray(new Ray());
      ray->vertical_samples = parseAttribute<unsigned int>(*vertical, "samples");
      ray->vertical_resolution = parseAttribute<double>(*vertical, "resolution");
      ray->vertical_min_angle = parseAttribute<double>(*vertical, "min_angle");
      ray->vertical_max_angle = parseAttribute<double>(*vertical, "max_angle");
      return ray.release();
    }
    catch (const std::exception &e)
    {
      CONSOLE_BRIDGE_logError("Ray horizontal: %s", e.what());
      return nullptr;
    }
  }
  return nullptr;
}

}
