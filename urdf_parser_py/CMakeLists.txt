cmake_minimum_required(VERSION 2.8.3)
project(urdf_parser_py)

find_package(catkin REQUIRED COMPONENTS rospy)
catkin_python_setup()
catkin_package()

install(PROGRAMS
    examples/display_urdf
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

