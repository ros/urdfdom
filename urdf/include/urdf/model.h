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

#ifndef ROBOT_MODEL_PARSER_H
#define ROBOT_MODEL_PARSER_H

#include <string>
#include <map>
#include <tinyxml/tinyxml.h>
#include <boost/function.hpp>
#include "link.h"


namespace urdf{

class Model
{
public:
  Model();

  /// \brief Load Model from TiXMLElement
  bool initXml(TiXmlElement *xml);
  /// \brief Load Model from TiXMLDocument
  bool initXml(TiXmlDocument *xml);
  /// \brief Load Model given a filename
  bool initFile(const std::string& filename);
  /// \brief Load Model given the name of a parameter on the parameter server
  bool initParam(const std::string& param);
  /// \brief Load Model from a XML-string
  bool initString(const std::string& xmlstring);

  boost::shared_ptr<const Link> getRoot(void) const{return this->root_link_;};
  boost::shared_ptr<const Link> getLink(const std::string& name) const;
  boost::shared_ptr<const Joint> getJoint(const std::string& name) const;
  const std::string& getName() const {return name_;};

  void getLinks(std::vector<boost::shared_ptr<Link> >& links) const;

  /// \brief get parent Link of a Link given name
  boost::shared_ptr<const Link> getParentLink(const std::string& name) const;
  /// \brief get parent Joint of a Link given name
  boost::shared_ptr<const Joint> getParentJoint(const std::string& name) const;
  /// \brief get child Link of a Link given name
  boost::shared_ptr<const Link> getChildLink(const std::string& name) const;
  /// \brief get child Joint of a Link given name
  boost::shared_ptr<const Joint> getChildJoint(const std::string& name) const;

  /// \brief complete list of Links
  std::map<std::string, boost::shared_ptr<Link> > links_;
  /// \brief complete list of Joints
  std::map<std::string, boost::shared_ptr<Joint> > joints_;
  /// \brief complete list of Materials
  std::map<std::string, boost::shared_ptr<Material> > materials_;

protected:
  void clear();

  std::string name_;

  /// non-const getLink()
  void getLink(const std::string& name,boost::shared_ptr<Link> &link) const;

  /// non-const getMaterial()
  boost::shared_ptr<Material> getMaterial(const std::string& name) const;

  /// in initXml(), onece all links are loaded,
  /// it's time to build a tree
  bool initTree(std::map<std::string, std::string> &parent_link_tree);

  /// in initXml(), onece tree is built,
  /// it's time to find the root Link
  bool initRoot(std::map<std::string, std::string> &parent_link_tree);

private:
  /// Model is restricted to a tree for now, which means there exists one root link
  ///  typically, root link is the world(inertial).  Where world is a special link
  /// or is the root_link_ the link attached to the world by PLANAR/FLOATING joint?
  ///  hmm...
  boost::shared_ptr<Link> root_link_;
  
  friend class ColladaModelReader;
};

}

#endif
