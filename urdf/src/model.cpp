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


Model::Model()
{
  this->clear();
}

void Model::clear()
{
  name_.clear();
  this->links_.clear();
  this->joints_.clear();
  this->materials_.clear();
  this->root_link_.reset();
}


bool Model::initFile(const std::string& filename)
{
  TiXmlDocument xml_doc;
  xml_doc.LoadFile(filename);

  return initXml(&xml_doc);
}


bool Model::initString(const std::string& xml_string)
{
  TiXmlDocument xml_doc;
  xml_doc.Parse(xml_string.c_str());

  return initXml(&xml_doc);
}


bool Model::initXml(TiXmlDocument *xml_doc)
{
  if (!xml_doc)
  {
    ROS_ERROR("Could not parse the xml");
    return false;
  }

  TiXmlElement *robot_xml = xml_doc->FirstChildElement("robot");
  if (!robot_xml)
  {
    ROS_ERROR("Could not find the 'robot' element in the xml file");
    return false;
  }
  return initXml(robot_xml);
}

bool Model::initXml(TiXmlElement *robot_xml)
{
  this->clear();

  ROS_DEBUG("Parsing robot xml");
  if (!robot_xml) return false;

  // Get robot name
  const char *name = robot_xml->Attribute("name");
  if (!name)
  {
    ROS_ERROR("No name given for the robot.");
    return false;
  }
  this->name_ = std::string(name);

  // Get all Material elements
  for (TiXmlElement* material_xml = robot_xml->FirstChildElement("material"); material_xml; material_xml = material_xml->NextSiblingElement("material"))
  {
    boost::shared_ptr<Material> material;
    material.reset(new Material);

    if (material->initXml(material_xml))
    {
      if (this->getMaterial(material->name))
      {
        ROS_ERROR("material '%s' is not unique.", material->name.c_str());
        material.reset();
        return false;
      }
      else
      {
        this->materials_.insert(make_pair(material->name,material));
        ROS_DEBUG("successfully added a new material '%s'", material->name.c_str());
      }
    }
    else
    {
      ROS_ERROR("material xml is not initialized correctly");
      material.reset();
      return false;
    }
  }

  // Get all Link elements
  for (TiXmlElement* link_xml = robot_xml->FirstChildElement("link"); link_xml; link_xml = link_xml->NextSiblingElement("link"))
  {
    boost::shared_ptr<Link> link;
    link.reset(new Link);

    if (link->initXml(link_xml))
    {
      if (this->getLink(link->name))
      {
        ROS_ERROR("link '%s' is not unique.", link->name.c_str());
        link.reset();
        return false;
      }
      else
      {
        // set link visual material
        ROS_DEBUG("setting link '%s' material", link->name.c_str());
        if (link->visual)
        {
          if (!link->visual->material_name.empty())
          {
            if (this->getMaterial(link->visual->material_name))
            {
              ROS_DEBUG("setting link '%s' material to '%s'", link->name.c_str(),link->visual->material_name.c_str());
              link->visual->material = this->getMaterial( link->visual->material_name.c_str() );
            }
            else
            {
              if (link->visual->material)
              {
                ROS_DEBUG("link '%s' material '%s' defined in Visual.", link->name.c_str(),link->visual->material_name.c_str());
                this->materials_.insert(make_pair(link->visual->material->name,link->visual->material));
              }
              else
              {
                ROS_ERROR("link '%s' material '%s' undefined.", link->name.c_str(),link->visual->material_name.c_str());
                link.reset();
                return false;
              }
            }
          }
        }

        this->links_.insert(make_pair(link->name,link));
        ROS_DEBUG("successfully added a new link '%s'", link->name.c_str());
      }
    }
    else
    {
      ROS_ERROR("link xml is not initialized correctly");
      link.reset();
      return false;
    }
  }
  // Get all Joint elements
  for (TiXmlElement* joint_xml = robot_xml->FirstChildElement("joint"); joint_xml; joint_xml = joint_xml->NextSiblingElement("joint"))
  {
    boost::shared_ptr<Joint> joint;
    joint.reset(new Joint);

    if (joint->initXml(joint_xml))
    {
      if (this->getJoint(joint->name))
      {
        ROS_ERROR("joint '%s' is not unique.", joint->name.c_str());
        joint.reset();
        return false;
      }
      else
      {
        this->joints_.insert(make_pair(joint->name,joint));
        ROS_DEBUG("successfully added a new joint '%s'", joint->name.c_str());
      }
    }
    else
    {
      ROS_ERROR("joint xml is not initialized correctly");
      joint.reset();
      return false;
    }
  }


  // every link has children links and joints, but no parents, so we create a
  // local convenience data structure for keeping child->parent relations
  std::map<std::string, std::string> parent_link_tree;
  parent_link_tree.clear();

  // building tree: name mapping
  if (!this->initTree(parent_link_tree))
  {
    ROS_ERROR("failed to build tree");
    return false;
  }

  // make sure tree is not empty
  if (parent_link_tree.empty()){
    ROS_ERROR("The robot xml does not contain any valid links. Are you parsing an empty file, or an un-processed xacro file?");
    return false;
  }

  return true;
}

bool Model::initTree(std::map<std::string, std::string> &parent_link_tree)
{
  this->root_link_.reset();

  // loop through all joints, for every link, assign children links and children joints
  for (std::map<std::string,boost::shared_ptr<Joint> >::iterator joint = this->joints_.begin();joint != this->joints_.end(); joint++)
  {
    std::string parent_link_name = joint->second->parent_link_name;
    std::string child_link_name = joint->second->child_link_name;

    ROS_DEBUG("    Build tree: joint: '%s' has parent link '%s' and child  link '%s'", joint->first.c_str(), parent_link_name.c_str(),child_link_name.c_str());
    
    // case where joint is not connected to any link
    if (parent_link_name.empty() && !child_link_name.empty())
    {
      ROS_ERROR("    Joint %s specifies only parent link but not child link",(joint->second)->name.c_str());
      return false;
    }
    else if (!parent_link_name.empty() &&  child_link_name.empty())
    {
      ROS_ERROR("    Joint %s specifies only child link but not parent link",(joint->second)->name.c_str());
      return false;
    }
    else if (parent_link_name.empty() && child_link_name.empty())
    {
      ROS_DEBUG("    Joint %s has not parent or child link, it is an abstract joint.",(joint->second)->name.c_str());
    }

    // normal joint case
    else
    {
      // find child and parent link of joint
      boost::shared_ptr<Link> child_link, parent_link;
      this->getLink(child_link_name, child_link);
      this->getLink(parent_link_name, parent_link);
      if (!child_link)
      {
        ROS_ERROR("    Child link '%s' of joint: %s not found",child_link_name.c_str(),joint->first.c_str());
        return false;
      }
      if (!parent_link)
      {
        if (root_link_)
        {
          ROS_ERROR("This tree contains two root links");
          return false; 
        }
        ROS_DEBUG("    Parent link '%s' of joint '%s' not found. This should be the root link.", parent_link_name.c_str(), joint->first.c_str() );
        parent_link.reset(new Link);
        parent_link->name = parent_link_name;
        root_link_ = parent_link;
        this->links_.insert(make_pair(parent_link_name, parent_link));
      }

      //set parent link for child link
      child_link->setParent(parent_link);

      //set parent joint for child link
      child_link->setParentJoint(joint->second);

      //set child joint for parent link
      parent_link->addChildJoint(joint->second);

      //set child link for parent link
      parent_link->addChild(child_link);

      // fill in child/parent string map
      parent_link_tree[child_link->name] = parent_link_name;

      ROS_DEBUG("    now Link '%s' has %i children ", parent_link->name.c_str(), (int)parent_link->child_links.size());
    }
  }

  // we should have found root
  if (!root_link_){
    ROS_ERROR("The tree does not contain a root link");
    return false;
  }

  return true;
}




boost::shared_ptr<Material> Model::getMaterial(const std::string& name) const
{
  boost::shared_ptr<Material> ptr;
  if (this->materials_.find(name) == this->materials_.end())
    ptr.reset();
  else
    ptr = this->materials_.find(name)->second;
  return ptr;
}

boost::shared_ptr<const Link> Model::getLink(const std::string& name) const
{
  boost::shared_ptr<const Link> ptr;
  if (this->links_.find(name) == this->links_.end())
    ptr.reset();
  else
    ptr = this->links_.find(name)->second;
  return ptr;
}

void Model::getLinks(std::vector<boost::shared_ptr<Link> >& links) const
{
  for (std::map<std::string,boost::shared_ptr<Link> >::const_iterator link = this->links_.begin();link != this->links_.end(); link++)
  {
    links.push_back(link->second);
  }
}

void Model::getLink(const std::string& name,boost::shared_ptr<Link> &link) const
{
  boost::shared_ptr<Link> ptr;
  if (this->links_.find(name) == this->links_.end())
    ptr.reset();
  else
    ptr = this->links_.find(name)->second;
  link = ptr;
}

boost::shared_ptr<const Joint> Model::getJoint(const std::string& name) const
{
  boost::shared_ptr<const Joint> ptr;
  if (this->joints_.find(name) == this->joints_.end())
    ptr.reset();
  else
    ptr = this->joints_.find(name)->second;
  return ptr;
}

}

