#include <urdf_parser/parser.h>
#include <ros/ros.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "memtest");
  while (ros::ok()){
    urdf::Parser urdf;

    TiXmlDocument xml_doc;
    xml_doc.LoadFile(std::string(argv[1]));
    urdf.init(&xml_doc);
  }
}
