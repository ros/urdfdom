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


#include <urdf_sensor/sensor.h>
#include <fstream>
#include <locale>
#include <sstream>
#include <stdexcept>
#include <string>
#include <algorithm>
#include <tinyxml2.h>
#include <console_bridge/console.h>

#include <urdf_parser/urdf_parser.h>

#include "./pose.hpp"
#include "./urdf_sensor.hpp"

namespace urdf{

bool parseCameraInternal(Camera &camera, tinyxml2::XMLElement* config)
{
  camera.clear();
  camera.type = VisualSensor::CAMERA;

  tinyxml2::XMLElement *image = config->FirstChildElement("image");
  if (image)
  {
    const char* width_char = image->Attribute("width");
    if (width_char)
    {
      try
      {
        camera.width = std::stoul(width_char);
      }
      catch (std::invalid_argument &e)
      {
        CONSOLE_BRIDGE_logError("Camera image width [%s] is not a valid int: %s", width_char, e.what());
        return false;
      }
      catch (std::out_of_range &e)
      {
        CONSOLE_BRIDGE_logError("Camera image width [%s] is out of range: %s", width_char, e.what());
        return false;
      }
    }
    else
    {
      CONSOLE_BRIDGE_logError("Camera sensor needs an image width attribute");
      return false;
    }

    const char* height_char = image->Attribute("height");
    if (height_char)
    {
      try
      {
        camera.height = std::stoul(height_char);
      }
      catch (std::invalid_argument &e)
      {
        CONSOLE_BRIDGE_logError("Camera image height [%s] is not a valid int: %s", height_char, e.what());
        return false;
      }
      catch (std::out_of_range &e)
      {
        CONSOLE_BRIDGE_logError("Camera image height [%s] is out of range: %s", height_char, e.what());
        return false;
      }
    }
    else
    {
      CONSOLE_BRIDGE_logError("Camera sensor needs an image height attribute");
      return false;
    }

    const char* format_char = image->Attribute("format");
    if (format_char)
      camera.format = std::string(format_char);
    else
    {
      CONSOLE_BRIDGE_logError("Camera sensor needs an image format attribute");
      return false;
    }

    const char* hfov_char = image->Attribute("hfov");
    if (hfov_char)
    {
      try {
        camera.hfov = strToDouble(hfov_char);
      } catch(std::runtime_error &) {
        CONSOLE_BRIDGE_logError("Camera image hfov [%s] is not a valid float", hfov_char);
        return false;
      }
    }
    else
    {
      CONSOLE_BRIDGE_logError("Camera sensor needs an image hfov attribute");
      return false;
    }

    const char* near_char = image->Attribute("near");
    if (near_char)
    {
      try {
        camera.near = strToDouble(near_char);
      } catch(std::runtime_error &) {
        CONSOLE_BRIDGE_logError("Camera image near [%s] is not a valid float", near_char);
        return false;
      }
    }
    else
    {
      CONSOLE_BRIDGE_logError("Camera sensor needs an image near attribute");
      return false;
    }

    const char* far_char = image->Attribute("far");
    if (far_char)
    {
      try {
        camera.far = strToDouble(far_char);
      } catch(std::runtime_error &) {
        CONSOLE_BRIDGE_logError("Camera image far [%s] is not a valid float", far_char);
        return false;
      }
    }
    else
    {
      CONSOLE_BRIDGE_logError("Camera sensor needs an image far attribute");
      return false;
    }

  }
  else
  {
    CONSOLE_BRIDGE_logError("Camera sensor has no <image> element");
    return false;
  }
  return true;
}

bool parseRayInternal(Ray &ray, tinyxml2::XMLElement* config)
{
  ray.clear();
  ray.type = VisualSensor::RAY;

  tinyxml2::XMLElement *horizontal = config->FirstChildElement("horizontal");
  if (horizontal)
  {
    const char* samples_char = horizontal->Attribute("samples");
    if (samples_char)
    {
      try
      {
        ray.horizontal_samples = std::stoul(samples_char);
      }
      catch (std::invalid_argument &e)
      {
        CONSOLE_BRIDGE_logError("Ray horizontal samples [%s] is not a valid float: %s", samples_char, e.what());
        return false;
      }
      catch (std::out_of_range &e)
      {
        CONSOLE_BRIDGE_logError("Ray horizontal samples [%s] is out of range: %s", samples_char, e.what());
        return false;
      }
    }

    const char* resolution_char = horizontal->Attribute("resolution");
    if (resolution_char)
    {
      try {
        ray.horizontal_resolution = strToDouble(resolution_char);
      } catch(std::runtime_error &) {
        CONSOLE_BRIDGE_logError("Ray horizontal resolution [%s] is not a valid float", resolution_char);
        return false;
      }
    }

    const char* min_angle_char = horizontal->Attribute("min_angle");
    if (min_angle_char)
    {
      try {
        ray.horizontal_min_angle = strToDouble(min_angle_char);
      } catch(std::runtime_error &) {
        CONSOLE_BRIDGE_logError("Ray horizontal min_angle [%s] is not a valid float", min_angle_char);
        return false;
      }
    }

    const char* max_angle_char = horizontal->Attribute("max_angle");
    if (max_angle_char)
    {
      try {
        ray.horizontal_max_angle = strToDouble(max_angle_char);
      } catch(std::runtime_error &) {
        CONSOLE_BRIDGE_logError("Ray horizontal max_angle [%s] is not a valid float", max_angle_char);
        return false;
      }
    }
  }

  tinyxml2::XMLElement *vertical = config->FirstChildElement("vertical");
  if (vertical)
  {
    const char* samples_char = vertical->Attribute("samples");
    if (samples_char)
    {
      try
      {
        ray.vertical_samples = std::stoul(samples_char);
      }
      catch (std::invalid_argument &e)
      {
        CONSOLE_BRIDGE_logError("Ray vertical samples [%s] is not a valid float: %s", samples_char, e.what());
        return false;
      }
      catch (std::out_of_range &e)
      {
        CONSOLE_BRIDGE_logError("Ray vertical samples [%s] is out of range: %s", samples_char, e.what());
        return false;
      }
    }

    const char* resolution_char = vertical->Attribute("resolution");
    if (resolution_char)
    {
      try {
        ray.vertical_resolution = strToDouble(resolution_char);
      } catch(std::runtime_error &) {
        CONSOLE_BRIDGE_logError("Ray vertical resolution [%s] is not a valid float", resolution_char);
        return false;
      }
    }

    const char* min_angle_char = vertical->Attribute("min_angle");
    if (min_angle_char)
    {
      try {
        ray.vertical_min_angle = strToDouble(min_angle_char);
      } catch(std::runtime_error &) {
        CONSOLE_BRIDGE_logError("Ray vertical min_angle [%s] is not a valid float", min_angle_char);
        return false;
      }
    }

    const char* max_angle_char = vertical->Attribute("max_angle");
    if (max_angle_char)
    {
      try {
        ray.vertical_max_angle = strToDouble(max_angle_char);
      } catch(std::runtime_error &) {
        CONSOLE_BRIDGE_logError("Ray vertical max_angle [%s] is not a valid float", max_angle_char);
        return false;
      }
    }
  }
  return false;
}

VisualSensorSharedPtr parseVisualSensor(tinyxml2::XMLElement *g)
{
  VisualSensorSharedPtr visual_sensor;

  // get sensor type
  tinyxml2::XMLElement *sensor_xml;
  if (g->FirstChildElement("camera"))
  {
    Camera *camera = new Camera();
    visual_sensor.reset(camera);
    sensor_xml = g->FirstChildElement("camera");
    if (!parseCameraInternal(*camera, sensor_xml))
      visual_sensor.reset();
  }
  else if (g->FirstChildElement("ray"))
  {
    Ray *ray = new Ray();
    visual_sensor.reset(ray);
    sensor_xml = g->FirstChildElement("ray");
    if (!parseRayInternal(*ray, sensor_xml))
      visual_sensor.reset();
  }
  else
  {
    CONSOLE_BRIDGE_logError("No know sensor types [camera|ray] defined in <sensor> block");
  }
  return visual_sensor;
}

}
