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


#include "urdf/link.h"
#include <ros/ros.h>

namespace urdf{

boost::shared_ptr<Geometry> parseGeometry(TiXmlElement *g)
{
  boost::shared_ptr<Geometry> geom;
  if (!g) return geom;

  TiXmlElement *shape = g->FirstChildElement();
  if (!shape)
  {
    ROS_ERROR("Geometry tag contains no child element.");
    return geom;
  }

  std::string type_name = shape->ValueStr();
  if (type_name == "sphere")
    geom.reset(new Sphere);
  else if (type_name == "box")
    geom.reset(new Box);
  else if (type_name == "cylinder")
    geom.reset(new Cylinder);
  else if (type_name == "mesh")
    geom.reset(new Mesh);
  else
  {
    ROS_ERROR("Unknown geometry type '%s'", type_name.c_str());
    return geom;
  }

  if (!geom->initXml(shape))
    return geom;

  return geom;
}

bool Material::initXml(TiXmlElement *config)
{
  bool has_rgb = false;
  bool has_filename = false;

  this->clear();

  if (!config->Attribute("name"))
  {
    ROS_ERROR("Material must contain a name attribute");
    return false;
  }

  this->name = config->Attribute("name");

  // texture
  TiXmlElement *t = config->FirstChildElement("texture");
  if (t)
  {
    if (t->Attribute("filename"))
    {
      this->texture_filename = t->Attribute("filename");
      has_filename = true;
    }
    else
    {
      ROS_ERROR("texture has no filename for Material %s",this->name.c_str());
    }
  }

  // color
  TiXmlElement *c = config->FirstChildElement("color");
  if (c)
  {
    if (c->Attribute("rgba"))
    {
      if (!this->color.init(c->Attribute("rgba")))
      {
        ROS_ERROR("Material %s has malformed color rgba values.",this->name.c_str());
        this->color.clear();
        return false;
      }
      else
        has_rgb = true;
    }
    else
    {
      ROS_ERROR("Material %s color has no rgba",this->name.c_str());
    }
  }

  return (has_rgb || has_filename);
}

bool Inertial::initXml(TiXmlElement *config)
{
  this->clear();

  // Origin
  TiXmlElement *o = config->FirstChildElement("origin");
  if (!o)
  {
    ROS_INFO("Origin tag not present for inertial element, using default (Identity)");
    this->origin.clear();
  }
  else
  {
    if (!this->origin.initXml(o))
    {
      ROS_ERROR("Inertial has a malformed origin tag");
      this->origin.clear();
      return false;
    }
  }

  TiXmlElement *mass_xml = config->FirstChildElement("mass");
  if (!mass_xml)
  {
    ROS_ERROR("Inertial element must have mass element");
    return false;
  }
  if (!mass_xml->Attribute("value"))
  {
    ROS_ERROR("Inertial: mass element must have value attributes");
    return false;
  }
  mass = atof(mass_xml->Attribute("value"));

  TiXmlElement *inertia_xml = config->FirstChildElement("inertia");
  if (!inertia_xml)
  {
    ROS_ERROR("Inertial element must have inertia element");
    return false;
  }
  if (!(inertia_xml->Attribute("ixx") && inertia_xml->Attribute("ixy") && inertia_xml->Attribute("ixz") &&
        inertia_xml->Attribute("iyy") && inertia_xml->Attribute("iyz") &&
        inertia_xml->Attribute("izz")))
  {
    ROS_ERROR("Inertial: inertia element must have ixx,ixy,ixz,iyy,iyz,izz attributes");
    return false;
  }
  ixx  = atof(inertia_xml->Attribute("ixx"));
  ixy  = atof(inertia_xml->Attribute("ixy"));
  ixz  = atof(inertia_xml->Attribute("ixz"));
  iyy  = atof(inertia_xml->Attribute("iyy"));
  iyz  = atof(inertia_xml->Attribute("iyz"));
  izz  = atof(inertia_xml->Attribute("izz"));

  return true;
}

bool Visual::initXml(TiXmlElement *config)
{
  this->clear();

  // Origin
  TiXmlElement *o = config->FirstChildElement("origin");
  if (!o)
  {
    ROS_DEBUG("Origin tag not present for visual element, using default (Identity)");
    this->origin.clear();
  }
  else if (!this->origin.initXml(o))
  {
    ROS_ERROR("Visual has a malformed origin tag");
    this->origin.clear();
    return false;
  }

  // Geometry
  TiXmlElement *geom = config->FirstChildElement("geometry");
  geometry = parseGeometry(geom);
  if (!geometry)
  {
    ROS_ERROR("Malformed geometry for Visual element");
    return false;
  }

  // Material
  TiXmlElement *mat = config->FirstChildElement("material");
  if (!mat)
  {
    ROS_DEBUG("visual element has no material tag.");
  }
  else
  {
    // get material name
    if (!mat->Attribute("name"))
    {
      ROS_ERROR("Visual material must contain a name attribute");
      return false;
    }
    this->material_name = mat->Attribute("name");

    // try to parse material element in place
    this->material.reset(new Material);
    if (!this->material->initXml(mat))
    {
      ROS_DEBUG("Could not parse material element in Visual block, maybe defined outside.");
      this->material.reset();
    }
    else
    {
      ROS_DEBUG("Parsed material element in Visual block.");
    }
  }

  return true;
}

bool Collision::initXml(TiXmlElement* config)
{
  this->clear();

  // Origin
  TiXmlElement *o = config->FirstChildElement("origin");
  if (!o)
  {
    ROS_INFO("Origin tag not present for collision element, using default (Identity)");
    this->origin.clear();
  }
  else if (!this->origin.initXml(o))
  {
    ROS_ERROR("Collision has a malformed origin tag");
    this->origin.clear();
    return false;
  }

  // Geometry
  TiXmlElement *geom = config->FirstChildElement("geometry");
  geometry = parseGeometry(geom);
  if (!geometry)
  {
    ROS_ERROR("Malformed geometry for Collision element");
    return false;
  }

  return true;
}

bool Sphere::initXml(TiXmlElement *c)
{
  this->clear();

  this->type = SPHERE;
  if (!c->Attribute("radius"))
  {
    ROS_ERROR("Sphere shape must have a radius attribute");
    return false;
  }

  radius = atof(c->Attribute("radius"));
  return false;
}

bool Box::initXml(TiXmlElement *c)
{
  this->clear();

  this->type = BOX;
  if (!c->Attribute("size"))
  {
    ROS_ERROR("Box shape has no size attribute");
    return false;
  }
  if (!dim.init(c->Attribute("size")))
  {
    ROS_ERROR("Box shape has malformed size attribute");
    dim.clear();
    return false;
  }
  return true;
}

bool Cylinder::initXml(TiXmlElement *c)
{
  this->clear();

  this->type = CYLINDER;
  if (!c->Attribute("length") ||
      !c->Attribute("radius"))
  {
    ROS_ERROR("Cylinder shape must have both length and radius attributes");
    return false;
  }

  length = atof(c->Attribute("length"));
  radius = atof(c->Attribute("radius"));
  return true;
}

bool Mesh::initXml(TiXmlElement *c)
{
  this->clear();

  this->type = MESH;
  if (!c->Attribute("filename"))
  {
    ROS_ERROR("Mesh must contain a filename attribute");
    return false;
  }

  filename = c->Attribute("filename");

  if (c->Attribute("scale"))
  {
    if (!this->scale.init(c->Attribute("scale")))
    {
      ROS_ERROR("Mesh scale was specified, but could not be parsed");
      this->scale.clear();
      return false;
    }
  }
  else
    ROS_DEBUG("Mesh scale was not specified, default to (1,1,1)");

  return true;
}


bool Link::initXml(TiXmlElement* config)
{
  this->clear();

  const char *name_char = config->Attribute("name");
  if (!name_char)
  {
    ROS_ERROR("No name given for the link.");
    return false;
  }
  name = std::string(name_char);

  // Inertial
  TiXmlElement *i = config->FirstChildElement("inertial");
  if (i)
  {
    inertial.reset(new Inertial);
    if (!inertial->initXml(i))
    {
      ROS_ERROR("Could not parse inertial element for Link '%s'", this->name.c_str());
      inertial.reset();
    }
  }

  // Visual
  TiXmlElement *v = config->FirstChildElement("visual");
  if (v)
  {
    visual.reset(new Visual);
    if (!visual->initXml(v))
    {
      ROS_ERROR("Could not parse visual element for Link '%s'", this->name.c_str());
      visual.reset();
    }
  }

  // Collision
  TiXmlElement *col = config->FirstChildElement("collision");
  if (col)
  {
    collision.reset(new Collision);
    if (!collision->initXml(col))
    {
      ROS_ERROR("Could not parse collision element for Link '%s'", this->name.c_str());
      collision.reset();
    }
  }

  return true;
}

void Link::setParent(boost::shared_ptr<Link> parent)
{
  this->parent_link = parent;
  ROS_DEBUG("set parent Link '%s' for Link '%s'", parent->name.c_str(), this->name.c_str());
}

void Link::setParentJoint(boost::shared_ptr<Joint> parent)
{
  this->parent_joint = parent;
  ROS_DEBUG("set parent joint '%s' to Link '%s'",  parent->name.c_str(), this->name.c_str());
}

void Link::addChild(boost::shared_ptr<Link> child)
{
  this->child_links.push_back(child);
  ROS_DEBUG("added child Link '%s' to Link '%s'",  child->name.c_str(), this->name.c_str());
}

void Link::addChildJoint(boost::shared_ptr<Joint> child)
{
  this->child_joints.push_back(child);
  ROS_DEBUG("added child Joint '%s' to Link '%s'", child->name.c_str(), this->name.c_str());
}



}

