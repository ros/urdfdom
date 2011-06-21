#include "urdf_parser/urdf_parser.h"
#include <ros/ros.h>
#include <fstream>
#include <iostream>

int main(int argc, char** argv){
  ros::init(argc, argv, "memtest");
  while (ros::ok()){
    std::string xml_string;
    std::fstream xml_file(argv[1], std::fstream::in);
    while ( xml_file.good() )
    {
      std::string line;
      std::getline( xml_file, line);
      xml_string += (line + "\n");
    }
    xml_file.close();


    urdf::parseURDF(xml_string);
  }
}
