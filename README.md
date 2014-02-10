urdfdom
===========

The URDF (U-Robot Description Format) library provides core data structures and a simple XML parsers for populating the class data structures from an URDF file.

For now, the details of the URDF specifications reside on http://ros.org/wiki/urdf
  
### Build Status
[![Build Status](https://travis-ci.org/ros/urdfdom.png)](https://travis-ci.org/ros/urdfdom)

### Installing from Source with ROS
This is not best practice for installing, but works. This version is for ROS Hydro but should be easily customized for future version of ROS:

```
sudo mv /opt/ros/hydro/include/urdf_parser/ /opt/ros/hydro/include/_urdf_parser/
sudo mv /opt/ros/hydro/lib/liburdfdom_model.so /opt/ros/hydro/lib/_liburdfdom_model.so
sudo mv /opt/ros/hydro/lib/liburdfdom_model_state.so /opt/ros/hydro/lib/_liburdfdom_model_state.so
sudo mv /opt/ros/hydro/lib/liburdfdom_sensor.so /opt/ros/hydro/lib/_liburdfdom_sensor.so
sudo mv /opt/ros/hydro/lib/liburdfdom_world.so /opt/ros/hydro/lib/_liburdfdom_world.so
sudo mv /opt/ros/hydro/lib/pkgconfig/urdfdom.pc /opt/ros/hydro/lib/pkgconfig/_urdfdom.pc
sudo mv /opt/ros/hydro/share/urdfdom/cmake/urdfdom-config.cmake /opt/ros/hydro/share/urdfdom/cmake/_urdfdom-config.cmake

cd ~/ros/urdfdom # where the git repo was checked out
mkdir -p build
cd build
cmake ../ -DCMAKE_INSTALL_PREFIX=/opt/ros/hydro
make -j8
sudo make install

# now rebuild your catkin workspace
cd ~/ros/ws_catkin
catkin_make
```
