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
#include <urdf_sensor/camera.h>
#include <urdf_sensor/ray.h>

#include <fstream>
#include <locale>
#include <sstream>
#include <stdexcept>
#include <string>
#include <algorithm>
#include <tinyxml.h>
#include <console_bridge/console.h>
#include "urdf_parser/pose.h"

namespace urdf {

SensorBaseSharedPtr CameraParser::parse(TiXmlElement &config)
{
  CameraSharedPtr camera(new Camera());

  TiXmlElement *image = config.FirstChildElement("image");
  if (image)
  {
    const char* width_char = image->Attribute("width");
    if (width_char)
    {
      try
      {
        camera->width = std::stoul(width_char);
      }
      catch (std::invalid_argument &e)
      {
        CONSOLE_BRIDGE_logError("Camera image width [%s] is not a valid int: %s", width_char, e.what());
        return CameraSharedPtr();
      }
      catch (std::out_of_range &e)
      {
        CONSOLE_BRIDGE_logError("Camera image width [%s] is out of range: %s", width_char, e.what());
        return CameraSharedPtr();
      }
    }
    else
    {
      CONSOLE_BRIDGE_logError("Camera sensor needs an image width attribute");
      return CameraSharedPtr();
    }

    const char* height_char = image->Attribute("height");
    if (height_char)
    {
      try
      {
        camera->height = std::stoul(height_char);
      }
      catch (std::invalid_argument &e)
      {
        CONSOLE_BRIDGE_logError("Camera image height [%s] is not a valid int: %s", height_char, e.what());
        return CameraSharedPtr();
      }
      catch (std::out_of_range &e)
      {
        CONSOLE_BRIDGE_logError("Camera image height [%s] is out of range: %s", height_char, e.what());
        return CameraSharedPtr();
      }
    }
    else
    {
      CONSOLE_BRIDGE_logError("Camera sensor needs an image height attribute");
      return CameraSharedPtr();
    }

    const char* format_char = image->Attribute("format");
    if (format_char)
      camera->format = std::string(format_char);
    else
    {
      CONSOLE_BRIDGE_logError("Camera sensor needs an image format attribute");
      return CameraSharedPtr();
    }

    const char* hfov_char = image->Attribute("hfov");
    if (hfov_char)
    {
      try {
        camera->hfov = strToDouble(hfov_char);
      } catch(std::runtime_error &) {
        CONSOLE_BRIDGE_logError("Camera image hfov [%s] is not a valid float", hfov_char);
        return CameraSharedPtr();
      }
    }
    else
    {
      CONSOLE_BRIDGE_logError("Camera sensor needs an image hfov attribute");
      return CameraSharedPtr();
    }

    const char* near_char = image->Attribute("near");
    if (near_char)
    {
      try {
        camera->near = strToDouble(near_char);
      } catch(std::runtime_error &) {
        CONSOLE_BRIDGE_logError("Camera image near [%s] is not a valid float", near_char);
        return CameraSharedPtr();
      }
    }
    else
    {
      CONSOLE_BRIDGE_logError("Camera sensor needs an image near attribute");
      return CameraSharedPtr();
    }

    const char* far_char = image->Attribute("far");
    if (far_char)
    {
      try {
        camera->far = strToDouble(far_char);
      } catch(std::runtime_error &) {
        CONSOLE_BRIDGE_logError("Camera image far [%s] is not a valid float", far_char);
        return CameraSharedPtr();
      }
    }
    else
    {
      CONSOLE_BRIDGE_logError("Camera sensor needs an image far attribute");
      return CameraSharedPtr();
    }

  }
  else
  {
    CONSOLE_BRIDGE_logError("Camera sensor has no <image> element");
    return CameraSharedPtr();
  }
  return camera;
}


SensorBaseSharedPtr RayParser::parse(TiXmlElement &config)
{
  RaySharedPtr ray (new Ray());

  TiXmlElement *horizontal = config.FirstChildElement("horizontal");
  if (horizontal)
  {
    const char* samples_char = horizontal->Attribute("samples");
    if (samples_char)
    {
      try
      {
        ray->horizontal_samples = std::stoul(samples_char);
      }
      catch (std::invalid_argument &e)
      {
        CONSOLE_BRIDGE_logError("Ray horizontal samples [%s] is not a valid float: %s", samples_char, e.what());
        return RaySharedPtr();
      }
      catch (std::out_of_range &e)
      {
        CONSOLE_BRIDGE_logError("Ray horizontal samples [%s] is out of range: %s", samples_char, e.what());
        return RaySharedPtr();
      }
    }

    const char* resolution_char = horizontal->Attribute("resolution");
    if (resolution_char)
    {
      try {
        ray->horizontal_resolution = strToDouble(resolution_char);
      } catch(std::runtime_error &) {
        CONSOLE_BRIDGE_logError("Ray horizontal resolution [%s] is not a valid float", resolution_char);
        return RaySharedPtr();
      }
    }

    const char* min_angle_char = horizontal->Attribute("min_angle");
    if (min_angle_char)
    {
      try {
        ray->horizontal_min_angle = strToDouble(min_angle_char);
      } catch(std::runtime_error &) {
        CONSOLE_BRIDGE_logError("Ray horizontal min_angle [%s] is not a valid float", min_angle_char);
        return RaySharedPtr();
      }
    }

    const char* max_angle_char = horizontal->Attribute("max_angle");
    if (max_angle_char)
    {
      try {
        ray->horizontal_max_angle = strToDouble(max_angle_char);
      } catch(std::runtime_error &) {
        CONSOLE_BRIDGE_logError("Ray horizontal max_angle [%s] is not a valid float", max_angle_char);
        return RaySharedPtr();
      }
    }
  }

  TiXmlElement *vertical = config.FirstChildElement("vertical");
  if (vertical)
  {
    const char* samples_char = vertical->Attribute("samples");
    if (samples_char)
    {
      try
      {
        ray->vertical_samples = std::stoul(samples_char);
      }
      catch (std::invalid_argument &e)
      {
        CONSOLE_BRIDGE_logError("Ray vertical samples [%s] is not a valid float: %s", samples_char, e.what());
        return RaySharedPtr();
      }
      catch (std::out_of_range &e)
      {
        CONSOLE_BRIDGE_logError("Ray vertical samples [%s] is out of range: %s", samples_char, e.what());
        return RaySharedPtr();
      }
    }

    const char* resolution_char = vertical->Attribute("resolution");
    if (resolution_char)
    {
      try {
        ray->vertical_resolution = strToDouble(resolution_char);
      } catch(std::runtime_error &) {
        CONSOLE_BRIDGE_logError("Ray vertical resolution [%s] is not a valid float", resolution_char);
        return RaySharedPtr();
      }
    }

    const char* min_angle_char = vertical->Attribute("min_angle");
    if (min_angle_char)
    {
      try {
        ray->vertical_min_angle = strToDouble(min_angle_char);
      } catch(std::runtime_error &) {
        CONSOLE_BRIDGE_logError("Ray vertical min_angle [%s] is not a valid float", min_angle_char);
        return RaySharedPtr();
      }
    }

    const char* max_angle_char = vertical->Attribute("max_angle");
    if (max_angle_char)
    {
      try {
        ray->vertical_max_angle = strToDouble(max_angle_char);
      } catch(std::runtime_error &) {
        CONSOLE_BRIDGE_logError("Ray vertical max_angle [%s] is not a valid float", max_angle_char);
        return RaySharedPtr();
      }
    }
  }
  return ray;
}

}
