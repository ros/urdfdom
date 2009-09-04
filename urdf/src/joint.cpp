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

#include <urdf/joint.h>
#include <ros/ros.h>

namespace urdf{

bool JointDynamics::initXml(TiXmlElement* config)
{
  this->clear();

  // Get joint damping
  const char* damping_str = config->Attribute("damping");
  if (damping_str == NULL)
    ROS_DEBUG("joint dynamics: no damping");
  else
    this->damping = atof(damping_str);

  // Get joint friction
  const char* friction_str = config->Attribute("friction");
  if (friction_str == NULL)
    ROS_DEBUG("joint dynamics: no friction");
  else
    this->friction = atof(friction_str);

  if (damping_str == NULL && friction_str == NULL)
  {
    ROS_ERROR("joint dynamics element specified with no damping and no friction");
    return false;
  }
  else{
    ROS_DEBUG("joint dynamics: damping %f and friction %f", damping, friction);
    return true;
  }
}

bool JointLimits::initXml(TiXmlElement* config)
{
  this->clear();

  // Get lower joint limit
  const char* lower_str = config->Attribute("lower");
  if (lower_str == NULL)
    ROS_DEBUG("joint limit: no lower");
  else
    this->lower = atof(lower_str);

  // Get upper joint limit
  const char* upper_str = config->Attribute("upper");
  if (upper_str == NULL)
    ROS_DEBUG("joint limit: no upper");
  else
    this->upper = atof(upper_str);

  // Get joint effort limit
  const char* effort_str = config->Attribute("effort");
  if (effort_str == NULL)
    ROS_DEBUG("joint limit: no effort");
  else
    this->effort = atof(effort_str);

  // Get joint velocity limit
  const char* velocity_str = config->Attribute("velocity");
  if (velocity_str == NULL)
    ROS_DEBUG("joint limit: no velocity");
  else
    this->velocity = atof(velocity_str);

  if (lower_str == NULL && upper_str == NULL && effort_str == NULL && velocity_str == NULL)
  {
    ROS_ERROR("joint limit element specified with no readable attributes");
    return false;
  }
  else
    return true;
}

bool JointSafety::initXml(TiXmlElement* config)
{
  this->clear();

  // Get soft_lower_limit joint limit
  const char* soft_lower_limit_str = config->Attribute("soft_lower_limit");
  if (soft_lower_limit_str == NULL)
  {
    ROS_DEBUG("joint safety: no soft_lower_limit, using default value");
    this->soft_lower_limit = 0;
  }
  else
    this->soft_lower_limit = atof(soft_lower_limit_str);

  // Get soft_upper_limit joint limit
  const char* soft_upper_limit_str = config->Attribute("soft_upper_limit");
  if (soft_upper_limit_str == NULL)
  {
    ROS_DEBUG("joint safety: no soft_upper_limit, using default value");
    this->soft_upper_limit = 0;
  }
  else
    this->soft_upper_limit = atof(soft_upper_limit_str);

  // Get k_position_ safety "position" gain - not exactly position gain
  const char* k_position_str = config->Attribute("k_position");
  if (k_position_str == NULL)
  {
    ROS_DEBUG("joint safety: no k_position, using default value");
    this->k_position = 0;
  }
  else
    this->k_position = atof(k_position_str);
  // Get k_velocity_ safety velocity gain
  const char* k_velocity_str = config->Attribute("k_velocity");
  if (k_velocity_str == NULL)
  {
    ROS_DEBUG("joint safety: no k_velocity, using default value");
    this->k_velocity = 0;
  }
  else
    this->k_velocity = atof(k_velocity_str);

  return true;
}

bool JointCalibration::initXml(TiXmlElement* config)
{
  this->clear();

  // Get reference_position
  const char* reference_position_str = config->Attribute("reference_position");
  if (reference_position_str == NULL)
  {
    ROS_DEBUG("joint calibration: no reference_position, using default value");
    this->reference_position = 0;
  }
  else
    this->reference_position = atof(reference_position_str);

  return true;
}

bool JointMimic::initXml(TiXmlElement* config)
{
  this->clear();

  // Get name of joint to mimic
  const char* joint_name_str = config->Attribute("joint");
  if (joint_name_str == NULL)
  {
    ROS_WARN("joint mimic: no mimic joint specified");
    return false;
  }
  else
    this->joint_name = joint_name_str;

  // Get mimic multiplier
  const char* multiplier_str = config->Attribute("multiplier");
  if (multiplier_str == NULL)
  {
    ROS_DEBUG("joint mimic: no multiplier, using default value of 1");
    this->multiplier = 1;
  }
  else
    this->multiplier = atof(multiplier_str);

  // Get mimic offset
  const char* offset_str = config->Attribute("offset");
  if (offset_str == NULL)
  {
    ROS_DEBUG("joint mimic: no offset, using default value of 0");
    this->offset = 0;
  }
  else
    this->offset = atof(offset_str);

  return true;
}

bool Joint::initXml(TiXmlElement* config)
{
  this->clear();

  // Get Joint Name
  const char *name = config->Attribute("name");
  if (!name)
  {
    ROS_ERROR("unnamed joint found");
    return false;
  }
  this->name = name;

  // Get transform from Parent Link to Joint Frame
  TiXmlElement *origin_xml = config->FirstChildElement("origin");
  if (!origin_xml)
  {
    ROS_DEBUG("Joint '%s' missing origin tag under parent describing transform from Parent Link to Joint Frame, (using Identity transform).", this->name.c_str());
    this->parent_to_joint_origin_transform.clear();
  }
  else
  {
    if (!this->parent_to_joint_origin_transform.initXml(origin_xml))
    {
      ROS_ERROR("Malformed parent origin element for joint '%s'", this->name.c_str());
      this->parent_to_joint_origin_transform.clear();
      return false;
    }
  }

  // Get Parent Link
  TiXmlElement *parent_xml = config->FirstChildElement("parent");
  if (parent_xml)
  {
    const char *pname = parent_xml->Attribute("link");
    if (!pname)
      ROS_INFO("no parent link name specified for Joint link '%s'. this might be the root?", this->name.c_str());
    else
    {
      this->parent_link_name = std::string(pname);

    }
  }

  // Get Child Link
  TiXmlElement *child_xml = config->FirstChildElement("child");
  if (child_xml)
  {
    const char *pname = child_xml->Attribute("link");
    if (!pname)
      ROS_INFO("no child link name specified for Joint link '%s'.", this->name.c_str());
    else
    {
      this->child_link_name = std::string(pname);

    }
  }

  // Get Joint type
  const char* type_char = config->Attribute("type");
  if (!type_char)
  {
    ROS_ERROR("joint '%s' has no type, check to see if it's a reference.", this->name.c_str());
    return false;
  }
  std::string type_str = type_char;
  if (type_str == "planar")
    type = PLANAR;
  else if (type_str == "floating")
    type = FLOATING;
  else if (type_str == "revolute")
    type = REVOLUTE;
  else if (type_str == "continuous")
    type = CONTINUOUS;
  else if (type_str == "prismatic")
    type = PRISMATIC;
  else if (type_str == "fixed")
    type = FIXED;
  else
  {
    ROS_ERROR("Joint '%s' has no known type '%s'", this->name.c_str(), type_str.c_str());
    return false;
  }

  // Get Joint Axis
  if (this->type != FLOATING)
  {
    // axis
    TiXmlElement *axis_xml = config->FirstChildElement("axis");
    if (axis_xml)
    {
      if (!axis_xml->Attribute("xyz"))
        ROS_INFO("no xyz attribute for axis element for Joint link '%s', using default values", this->name.c_str());
      else
      {
        if (!this->axis.init(axis_xml->Attribute("xyz")))
        {
          if (this->type == PLANAR)
            ROS_DEBUG("PLANAR Joint '%s' will require an axis tag in the future which indicates the surface normal of the plane.", this->name.c_str());
          else
          {
            ROS_ERROR("Malformed axis element for joint '%s'", this->name.c_str());
            this->axis.clear();
            return false;
          }
        }
      }
    }
  }

  // Get limit
  TiXmlElement *limit_xml = config->FirstChildElement("limit");
  if (limit_xml)
  {
    limits.reset(new JointLimits);
    if (!limits->initXml(limit_xml))
    {
      ROS_ERROR("Could not parse limit element for joint '%s'", this->name.c_str());
      limits.reset();
    }
  }

  // Get safety
  TiXmlElement *safety_xml = config->FirstChildElement("safety_controller");
  if (safety_xml)
  {
    safety.reset(new JointSafety);
    if (!safety->initXml(safety_xml))
    {
      ROS_ERROR("Could not parse safety element for joint '%s'", this->name.c_str());
      safety.reset();
    }
  }

  // Get calibration
  TiXmlElement *calibration_xml = config->FirstChildElement("calibration");
  if (calibration_xml)
  {
    calibration.reset(new JointCalibration);
    if (!calibration->initXml(calibration_xml))
    {
      ROS_ERROR("Could not parse calibration element for joint  '%s'", this->name.c_str());
      calibration.reset();
    }
  }

  // Get Joint Mimic
  TiXmlElement *mimic_xml = config->FirstChildElement("mimic");
  if (mimic_xml)
  {
    mimic.reset(new JointMimic);
    if (!mimic->initXml(mimic_xml))
    {
      ROS_WARN("Could not parse mimic element for joint  '%s'", this->name.c_str());
      mimic.reset();
    }
  }

  // Get Dynamics
  TiXmlElement *prop_xml = config->FirstChildElement("dynamics");
  if (prop_xml)
  {
    dynamics.reset(new JointDynamics);
    if (!dynamics->initXml(prop_xml))
    {
      ROS_ERROR("Could not parse joint_dynamics element for joint '%s'", this->name.c_str());
      dynamics.reset();
    }
  }

  return true;
}



}
