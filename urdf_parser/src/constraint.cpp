/*********************************************************************
* Software Ligcense Agreement (BSD License)
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

/* Author: Fabian Finkbeiner */

#include <locale>
#include <sstream>
#include <stdexcept>
#include <string>
#include <urdf_model/constraint.hpp>
#include <console_bridge/console.h>
#include <tinyxml2.h>
#include <urdf_parser/urdf_parser.hpp>

#include "./pose.hpp"

namespace urdf{


bool parseConstraint(Constraint &constraint, tinyxml2::XMLElement* config,
                const urdf_export_helpers::URDFVersion version)
{
  constraint.clear();

  // Get Constraint Name
  const char *name = config->Attribute("name");
  if (!name)
  {
    CONSOLE_BRIDGE_logError("unnamed constraint found");
    return false;
  }
  constraint.name = name;

  // Get transform from Parent Link to Constraint Parent Frame
  tinyxml2::XMLElement *parent_origin_xml = config->FirstChildElement("parent_origin");
  if (!parent_origin_xml)
  {
    CONSOLE_BRIDGE_logDebug("urdfdom: Constraint [%s] missing parent_origin tag under parent describing transform from Parent Link to Constraint Parent Frame, (using Identity transform).", constraint.name.c_str());
    constraint.parent_to_constraint_parent_transform.clear();
  }
  else
  {
    if (!parsePoseInternal(constraint.parent_to_constraint_parent_transform, parent_origin_xml, version))
    {
      constraint.parent_to_constraint_parent_transform.clear();
      CONSOLE_BRIDGE_logError("Malformed parent origin element for constraint [%s]", constraint.name.c_str());
      return false;
    }
  }


    // Get transform from Child Link to Constraint Child Frame
  tinyxml2::XMLElement *child_origin_xml = config->FirstChildElement("child_origin");
  if (!child_origin_xml)
  {
    CONSOLE_BRIDGE_logDebug("urdfdom: Constraint [%s] missing child_origin tag under parent describing transform from Child Link to Constraint Child Frame, (using Identity transform).", constraint.name.c_str());
    constraint.child_to_constraint_child_transform.clear();
  }
  else
  {
    if (!parsePoseInternal(constraint.child_to_constraint_child_transform, child_origin_xml, version))
    {
      constraint.child_to_constraint_child_transform.clear();
      CONSOLE_BRIDGE_logError("Malformed child origin element for constraint [%s]", constraint.name.c_str());
      return false;
    }
  }

  // Get Parent Link
  tinyxml2::XMLElement *parent_xml = config->FirstChildElement("parent");
  if (parent_xml)
  {
    const char *pname = parent_xml->Attribute("link");
    if (!pname)
    {
      CONSOLE_BRIDGE_logInform("no parent link name specified for Constraint parent_link [%s]", constraint.name.c_str());
    }
    else
    {
      constraint.parent_link_name = std::string(pname);
    }
  }

  // Get Child Link
  tinyxml2::XMLElement *child_xml = config->FirstChildElement("child");
  if (child_xml)
  {
    const char *pname = child_xml->Attribute("link");
    if (!pname)
    {
      CONSOLE_BRIDGE_logInform("no child link name specified for Constraint child_link [%s].", constraint.name.c_str());
    }
    else
    {
      constraint.child_link_name = std::string(pname);
    }
  }

  // Get Constraint type
  const char* type_char = config->Attribute("type");
  if (!type_char)
  {
    CONSOLE_BRIDGE_logError("constraint [%s] has no type, check to see if it's a reference.", constraint.name.c_str());
    return false;
  }

  std::string type_str = type_char;
  if (type_str == "revolute" || type_str == "revolute_joint")
    constraint.type = Constraint::REVOLUTE;
  else if (type_str == "prismatic" || type_str == "prismatic_joint")
    constraint.type = Constraint::PRISMATIC;
  else if (type_str == "universal" || type_str == "universal_joint")
    constraint.type = Constraint::UNIVERSAL;
  else if (type_str == "spherical" || type_str == "spherical_joint")
    constraint.type = Constraint::SPHERICAL;
  else if (type_str == "link")
    constraint.type = Constraint::LINK;
  else
  {
    CONSOLE_BRIDGE_logError("Constraint [%s] has no known type [%s]", constraint.name.c_str(), type_str.c_str());
    return false;
  }


   // Get Constraint parent Axis
  if (constraint.type != Constraint::LINK && constraint.type != Constraint::SPHERICAL)
  {
    // axis
    tinyxml2::XMLElement *parent_axis_xml = config->FirstChildElement("parent_axis");
    if (!parent_axis_xml){
      CONSOLE_BRIDGE_logDebug("urdfdom: no parent_axis element for Constraint [%s], defaulting to (1,0,0) axis", constraint.name.c_str());
      constraint.parent_axis = Vector3(1.0, 0.0, 0.0);
    }
    else{
      if (parent_axis_xml->Attribute("xyz")){
        try {
          constraint.parent_axis.init(parent_axis_xml->Attribute("xyz"));
        }
        catch (ParseError &e) {
          constraint.parent_axis.clear();
          CONSOLE_BRIDGE_logError("Malformed axis element for constraint [%s]: %s", constraint.name.c_str(), e.what());
          return false;
        }
      }
    }
  }

  // Get Constraint child Axis
  if (constraint.type == Constraint::UNIVERSAL)
  {
    // axis
    tinyxml2::XMLElement *child_axis_xml = config->FirstChildElement("child_axis");
    if (!child_axis_xml){
      CONSOLE_BRIDGE_logDebug("urdfdom: no child_axis element for Constraint [%s], defaulting to (1,0,0) axis", constraint.name.c_str());
      constraint.child_axis = Vector3(1.0, 0.0, 0.0);
    }
    else{
      if (child_axis_xml->Attribute("xyz")){
        try {
          constraint.child_axis.init(child_axis_xml->Attribute("xyz"));
        }
        catch (ParseError &e) {
          constraint.child_axis.clear();
          CONSOLE_BRIDGE_logError("Malformed axis element for constraint [%s]: %s", constraint.name.c_str(), e.what());
          return false;
        }
      }
    }
  }
  return true;
}


/* exports */
bool exportParentPose(Pose &pose, tinyxml2::XMLElement* xml)
{
  tinyxml2::XMLElement* origin = xml->GetDocument()->NewElement("parent_origin");
  std::string pose_xyz_str = urdf_export_helpers::values2str(pose.position);
  std::string pose_rpy_str = urdf_export_helpers::values2str(pose.rotation);
  origin->SetAttribute("xyz", pose_xyz_str.c_str());
  origin->SetAttribute("rpy", pose_rpy_str.c_str());
  xml->LinkEndChild(origin);
  return true;
}

bool exportChildPose(Pose &pose, tinyxml2::XMLElement* xml)
{
  tinyxml2::XMLElement* origin = xml->GetDocument()->NewElement("child_origin");
  std::string pose_xyz_str = urdf_export_helpers::values2str(pose.position);
  std::string pose_rpy_str = urdf_export_helpers::values2str(pose.rotation);
  origin->SetAttribute("xyz", pose_xyz_str.c_str());
  origin->SetAttribute("rpy", pose_rpy_str.c_str());
  xml->LinkEndChild(origin);
  return true;
}

bool exportConstraint(Constraint &constraint, tinyxml2::XMLElement* xml)
{
  tinyxml2::XMLElement * constraint_xml = xml->GetDocument()->NewElement("constraint");
  constraint_xml->SetAttribute("name", constraint.name.c_str());
  if (constraint.type == urdf::Constraint::REVOLUTE)
    constraint_xml->SetAttribute("type", "revolute_joint");
  else if (constraint.type == urdf::Constraint::PRISMATIC)
    constraint_xml->SetAttribute("type", "prismatic_joint");
  else if (constraint.type == urdf::Constraint::UNIVERSAL)
    constraint_xml->SetAttribute("type", "universal_joint");
  else if (constraint.type == urdf::Constraint::SPHERICAL)
    constraint_xml->SetAttribute("type", "spherical_joint");
  else if (constraint.type == urdf::Constraint::LINK)
    constraint_xml->SetAttribute("type", "link");
  else
    CONSOLE_BRIDGE_logError("ERROR:  Constraint [%s] type [%d] is not a defined type.\n",constraint.name.c_str(), constraint.type);

  // origins
  exportParentPose(constraint.parent_to_constraint_parent_transform, constraint_xml);
  exportChildPose(constraint.child_to_constraint_child_transform, constraint_xml);

  // axes
  if(constraint.type != Constraint::LINK && constraint.type!=Constraint::SPHERICAL)
  {
    tinyxml2::XMLElement * parent_axis_xml = constraint_xml->GetDocument()->NewElement("parent_axis");
    parent_axis_xml->SetAttribute("xyz", urdf_export_helpers::values2str(constraint.parent_axis).c_str());
    constraint_xml->LinkEndChild(parent_axis_xml);
  }

  if (constraint.type==Constraint::UNIVERSAL)
  {
    tinyxml2::XMLElement * child_axis_xml = constraint_xml->GetDocument()->NewElement("child_axis");
    child_axis_xml->SetAttribute("xyz", urdf_export_helpers::values2str(constraint.child_axis).c_str());
    constraint_xml->LinkEndChild(child_axis_xml);
  }
  // parent
  tinyxml2::XMLElement * parent_xml = constraint_xml->GetDocument()->NewElement("parent");
  parent_xml->SetAttribute("link", constraint.parent_link_name.c_str());
  constraint_xml->LinkEndChild(parent_xml);

  // child
  tinyxml2::XMLElement * child_xml = constraint_xml->GetDocument()->NewElement("child");
  child_xml->SetAttribute("link", constraint.child_link_name.c_str());
  constraint_xml->LinkEndChild(child_xml);

  xml->LinkEndChild(constraint_xml);
  return true;
}


}


