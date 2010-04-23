/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2010, Willow Garage, Inc.
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

#include "collada_urdf/ColladaWriter.h"

int main(int argc, char** argv)
{
    if (argc != 3) {
        std::cerr << "Usage: urdf_to_collada input.urdf output.dae" << std::endl;
        return -1;
    }

    std::string input_filename(argv[1]);
    std::string output_filename(argv[2]);

    TiXmlDocument robot_model_xml;
    if (!robot_model_xml.LoadFile(input_filename.c_str())) {
        std::cerr << "Error opening file " << argv[1] << std::endl;
        return -1;
    }

    TiXmlElement* robot_xml = robot_model_xml.FirstChildElement("robot");
    if (!robot_xml) {
        std::cerr << "Error parsing URDF model from XML" << std::endl;
        return -1;
    }

    urdf::Model robot;
    if (!robot.initXml(robot_xml)) {
        std::cerr << "Error parsing URDF model from XML" << std::endl;
        return -1;
    }

    collada_urdf::ColladaWriter writer(&robot, input_filename);
    if (!writer.writeDocument(output_filename)) {
        std::cerr << "Error writing document" << std::endl;
        return -1;
    }

    return 0;
}
