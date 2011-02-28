#include <urdf/model.h>
#include <ros/ros.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "memtest");
  while (ros::ok()){
    urdf::Model urdf;
    urdf.initFile(std::string(argv[1]));
  }
}
