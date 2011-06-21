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

#ifndef COLLADA_PARSER_COLLADA_PARSER_H
#define COLLADA_PARSER_COLLADA_PARSER_H

#include <string>
#include <map>
#include <boost/function.hpp>

#include <urdf_interface/model.h>


namespace urdf{

class ColladaParser : public ModelInterface
{
public:

  /// \brief Load Model from string
  bool initCollada(const std::string &xml_string );

protected:
  /// non-const get Collada Link()
  void getColladaLink(const std::string& name,boost::shared_ptr<Link> &link) const;

  /// non-const getColladaMaterial()
  //boost::shared_ptr<Material> getColladaMaterial(const std::string& name) const;

  /// in initXml(), onece all links are loaded,
  /// it's time to build a tree
  bool initColladaTree(std::map<std::string, std::string> &parent_link_tree);

  /// in initXml(), onece tree is built,
  /// it's time to find the root Link
  bool initColladaRoot(std::map<std::string, std::string> &parent_link_tree);

  friend class ColladaModelReader;
};

}

#endif
