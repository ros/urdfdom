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
#include <sstream>
#include <boost/lexical_cast.hpp>
#include <algorithm>
#include <urdf_parser/exceptions.h>
#include <urdf_parser/console.h>

namespace urdf{

boost::shared_ptr<VisualSensor> Sensor::parseVisualSensor(TiXmlElement *g)
{
  boost::shared_ptr<VisualSensor> visual_sensor;
  // get sensor type
  TiXmlElement *sensor_xml;
  if (g->FirstChildElement("camera"))
  {
    visual_sensor.reset(new Camera);
    sensor_xml = g->FirstChildElement("camera");
  }
  else if (g->FirstChildElement("ray"))
  {
    visual_sensor.reset(new Ray);
    sensor_xml = g->FirstChildElement("ray");
  }
  else
  {
    logError("No know sensor types [camera|ray] defined in <sensor> block");
    return visual_sensor;
  }

  // initialize sensor
  try {
    visual_sensor->initXml(sensor_xml);
  }
  catch (ParseError &e) {
    visual_sensor.reset();
    throw e.addMessage("failed to parse sensor");
  }
}

void Camera::initXml(TiXmlElement* config)
{
  this->clear();
  this->type = CAMERA;

  TiXmlElement *image = config->FirstChildElement("image");
  if (image)
  {
    const char* width_char = image->Attribute("width");
    if (width_char)
    {
      try
      {
        this->width = boost::lexical_cast<double>(width_char);
      }
      catch (boost::bad_lexical_cast &e)
      {
        std::stringstream stm;
        stm << "camera image width [" << width_char << "] is not a valid float";
        throw ParseError(stm.str());
      }
    }
    else
      throw ParseError("camera sensor needs an image width attribute");

    const char* height_char = image->Attribute("height");
    if (height_char)
    {
      try
      {
        this->height = boost::lexical_cast<double>(height_char);
      }
      catch (boost::bad_lexical_cast &e)
      {
        std::stringstream stm;
        stm << "camera image height [" << height_char << "] is not a valid float";
        throw ParseError(stm.str());
      }

    }
    else
      throw ParseError("camera sensor needs an image height attribute");

    const char* format_char = image->Attribute("format");
    if (format_char)
      this->format = std::string(format_char);
    else
      throw ParseError("camera sensor needs an image format attribute");

    const char* hfov_char = image->Attribute("hfov");
    if (hfov_char)
    {
      try
      {
        this->hfov = boost::lexical_cast<double>(hfov_char);
      }
      catch (boost::bad_lexical_cast &e)
      {
        std::stringstream stm;
        stm << "camera image hfov [" << hfov_char << "] is not a valid float";
        throw ParseError(stm.str());
      }

    }
    else
      throw ParseError("camera sensor needs an image hfov attribute");

    const char* near_char = image->Attribute("near");
    if (near_char)
    {
      try
      {
        this->near = boost::lexical_cast<double>(near_char);
      }
      catch (boost::bad_lexical_cast &e)
      {
        std::stringstream stm;
        stm << "camera image near [" << near_char << "] is not a valid float";
        throw ParseError(stm.str());
      }

    }
    else
      throw ParseError("camera sensor needs an image near attribute");

    const char* far_char = image->Attribute("far");
    if (far_char)
    {
      try
      {
        this->far = boost::lexical_cast<double>(far_char);
      }
      catch (boost::bad_lexical_cast &e)
      {
        std::stringstream stm;
        stm << "camera image far [" << far_char << "] is not a valid float";
        throw ParseError(stm.str());
      }

    }
    else
      throw ParseError("camera sensor needs an image far attribute");
  }
  else
  {
    throw ParseError("camera sensor has no <image> element");
  }
}

void Ray::initXml(TiXmlElement* config)
{
  this->clear();
  this->type = RAY;

  TiXmlElement *horizontal = config->FirstChildElement("horizontal");
  if (horizontal)
  {
    const char* samples_char = horizontal->Attribute("samples");
    if (samples_char)
    {
      try
      {
        this->horizontal_samples = boost::lexical_cast<double>(samples_char);
      }
      catch (boost::bad_lexical_cast &e)
      {
        std::stringstream stm;
        stm << "ray horizontal samples [" << samples_char << "] is not a valid float";
        throw ParseError(stm.str());
      }
    }

    const char* resolution_char = horizontal->Attribute("resolution");
    if (resolution_char)
    {
      try
      {
        this->horizontal_resolution = boost::lexical_cast<double>(resolution_char);
      }
      catch (boost::bad_lexical_cast &e)
      {
        std::stringstream stm;
        stm << "ray horizontal resolution [" << resolution_char << "] is not a valid float";
        throw ParseError(stm.str());
      }
    }

    const char* min_angle_char = horizontal->Attribute("min_angle");
    if (min_angle_char)
    {
      try
      {
        this->horizontal_min_angle = boost::lexical_cast<double>(min_angle_char);
      }
      catch (boost::bad_lexical_cast &e)
      {
        std::stringstream stm;
        stm << "ray horizontal min_angle [" << min_angle_char << "] is not a valid float";
        throw ParseError(stm.str());
      }
    }

    const char* max_angle_char = horizontal->Attribute("max_angle");
    if (max_angle_char)
    {
      try
      {
        this->horizontal_max_angle = boost::lexical_cast<double>(max_angle_char);
      }
      catch (boost::bad_lexical_cast &e)
      {
        std::stringstream stm;
        stm << "ray horizontal max_angle [" << max_angle_char << "] is not a valid float";
        throw ParseError(stm.str());
      }
    }
  }

  TiXmlElement *vertical = config->FirstChildElement("vertical");
  if (vertical)
  {
    const char* samples_char = vertical->Attribute("samples");
    if (samples_char)
    {
      try
      {
        this->vertical_samples = boost::lexical_cast<double>(samples_char);
      }
      catch (boost::bad_lexical_cast &e)
      {
        std::stringstream stm;
        stm << "ray vertical samples [" << samples_char << "] is not a valid float";
        throw ParseError(stm.str());
      }
    }

    const char* resolution_char = vertical->Attribute("resolution");
    if (resolution_char)
    {
      try
      {
        this->vertical_resolution = boost::lexical_cast<double>(resolution_char);
      }
      catch (boost::bad_lexical_cast &e)
      {
        std::stringstream stm;
        stm << "ray vertical resolution [" << resolution_char << "] is not a valid float";
        throw ParseError(stm.str());
      }
    }

    const char* min_angle_char = vertical->Attribute("min_angle");
    if (min_angle_char)
    {
      try
      {
        this->vertical_min_angle = boost::lexical_cast<double>(min_angle_char);
      }
      catch (boost::bad_lexical_cast &e)
      {
        std::stringstream stm;
        stm << "ray vertical min_angle [" << min_angle_char << "] is not a valid float";
        throw ParseError(stm.str());
      }
    }

    const char* max_angle_char = vertical->Attribute("max_angle");
    if (max_angle_char)
    {
      try
      {
        this->vertical_max_angle = boost::lexical_cast<double>(max_angle_char);
      }
      catch (boost::bad_lexical_cast &e)
      {
        std::stringstream stm;
        stm << "ray vertical max_angle [" << max_angle_char << "] is not a valid float";
        throw ParseError(stm.str());
      }
    }
  }

}


void Sensor::initXml(TiXmlElement* config)
{
  
  this->clear();

  const char *name_char = config->Attribute("name");
  if (!name_char)
  {
    throw ParseError("No name given for the sensor.");
  }
  this->name = std::string(name_char);

  // parse parent_link_name
  const char *parent_link_name_char = config->Attribute("parent_link_name");
  if (!parent_link_name_char)
  {
    throw ParseError("No parent_link_name given for the sensor.");
  }
  this->parent_link_name = std::string(parent_link_name_char);

  // parse origin
  TiXmlElement *o = config->FirstChildElement("origin");
  if (o)
  {
    try {
      this->origin.initXml(o);
    }
    catch (ParseError &e) {
      this->origin.clear();
      throw e.addMessage("sensor has malformed origin tag");
    }
  }

  // parse sensor
  this->sensor = this->parseVisualSensor(config);

}


void Sensor::setParent(boost::shared_ptr<Link> parent)
{
  this->parent_link_ = parent;
  logDebug("set parent Link '%s' for Link '%s'", parent->name.c_str(), this->name.c_str());
}

}


