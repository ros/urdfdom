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
*   * Redstributions of source code must retain the above copyright
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

#include "urdf_parser/urdf_parser.h"
#include <iostream>
#include <fstream>

using namespace urdf;
using namespace std;

void addChildLinkNames(LinkConstSharedPtr link, ofstream& os)
{
  os << "\"" << link->name << "\" [label=\"" << link->name << "\"];" << endl;
  for (std::vector<LinkSharedPtr>::const_iterator child = link->child_links.begin(); child != link->child_links.end(); child++)
    addChildLinkNames(*child, os);
}

void addChildJointNames(LinkConstSharedPtr link, ofstream& os, bool advanced_info)
{
  double r, p, y;
  std::string type_str;

  for (std::vector<LinkSharedPtr>::const_iterator child = link->child_links.begin(); child != link->child_links.end(); child++){
    (*child)->parent_joint->parent_to_joint_origin_transform.rotation.getRPY(r,p,y);
    if ((*child)->parent_joint->type == Joint::PLANAR)
      type_str = "planar";
    else if ((*child)->parent_joint->type == Joint::FLOATING)
      type_str = "floating";
    else if ((*child)->parent_joint->type == Joint::REVOLUTE)
      type_str = "revolute";
    else if ((*child)->parent_joint->type == Joint::CONTINUOUS)
      type_str = "continuous";
    else if ((*child)->parent_joint->type == Joint::PRISMATIC)
      type_str = "prismatic";
    else if ((*child)->parent_joint->type == Joint::FIXED)
      type_str = "fixed";
    else
      type_str = "UNKNOWN";
    os << "\"" << link->name << "\" -> \"" << (*child)->parent_joint->name
       << "\" [label=\"xyz: "
       << (*child)->parent_joint->parent_to_joint_origin_transform.position.x << " "
       << (*child)->parent_joint->parent_to_joint_origin_transform.position.y << " "
       << (*child)->parent_joint->parent_to_joint_origin_transform.position.z << " "
       << "\\nrpy: " << r << " " << p << " " << y << " ";
    if (advanced_info) {
      os << "\\ntype: " << type_str << " ";
      if ((*child)->parent_joint->mimic)
        os << "\\nmimics: " << (*child)->parent_joint->mimic->joint_name
           << "\\n(o: " << (*child)->parent_joint->mimic->offset
           << " m: " << (*child)->parent_joint->mimic->multiplier << ") ";
    }
    os << "\"]" << endl;
    os << "\"" << (*child)->parent_joint->name << "\" -> \"" << (*child)->name << "\"" << endl;
    addChildJointNames(*child, os, advanced_info);
  }
}


void printTree(LinkConstSharedPtr link, string file, bool advanced_info)
{
  std::ofstream os;
  os.open(file.c_str());
  os << "digraph G {" << endl;

  os << "node [shape=box];" << endl;
  addChildLinkNames(link, os);

  os << "node [shape=ellipse, color=blue, fontcolor=blue];" << endl;
  addChildJointNames(link, os, advanced_info);

  os << "}" << endl;
  os.close();
}



int main(int argc, char** argv)
{
  std::string usage = "Usage: urdf_to_graphiz [-a] input.xml [OUTPUT]\n"
                       "Will create either $ROBOT_NAME.gv & $ROBOT_NAME.pdf in CWD"
                       "or OUTPUT.gv & OUTPUT.pdf.\n"
                       "Options:\n"
                       "  -a   Provides advanced joint information\n";

  bool advanced_info = false;
  bool option = false;

  if (argc >= 2 && string(argv[1]).substr(0,1) == "-") {
    option = true;
    if (string(argv[1]) == "-a")
      advanced_info = true;
  }

  if (argc - option < 3 || (option && !advanced_info))
    std::cerr << usage;
  else if (argc - advanced_info < 2)
    return -1;

  // get the entire file
  std::string xml_string;
  std::fstream xml_file(argv[1+option], std::fstream::in);
  while ( xml_file.good() )
  {
    std::string line;
    std::getline( xml_file, line);
    xml_string += (line + "\n");
  }
  xml_file.close();

  ModelInterfaceSharedPtr robot = parseURDF(xml_string);
  if (!robot){
    std::cerr << "ERROR: Model Parsing the xml failed" << std::endl;
    return -1;
  }

  string output = robot->getName();
  if (argc == 3+option)
    output = argv[2+option];

  // print entire tree to file
  if (advanced_info)
    cout << "Giving advanced joint information..." << endl;
  printTree(robot->getRoot(), output+".gv", advanced_info);
  cout << "Created file " << output << ".gv" << endl;

  string command = "dot -Tpdf "+output+".gv  -o "+output+".pdf";
  if (system(command.c_str()) != -1)
    cout << "Created file " << output << ".pdf" << endl;
  else
    cout << "There was an error executing '" << command << "'" << endl;
  return 0;
}
