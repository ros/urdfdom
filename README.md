urdfdom
===========

The URDF (U-Robot Description Format) library provides core data structures and a simple XML parsers for populating the class data structures from an URDF file.

For now, the details of the URDF specifications reside on http://ros.org/wiki/urdf

### Using with ROS

If you choose to check this repository out for use with ROS, be aware that the necessary ``package.xml`` is not 
included in this repo but instead is added in during the ROS release process. To emulate this, pull the appropriate
file into this repository using the following format. Be sure to replace the ALLCAPS words with the appropriate terms:

```
wget https://raw.github.com/ros-gbp/urdfdom-release/debian/ROS_DISTRO/UBUNTU_DISTRO/urdfdom/package.xml
```

For example:
```
wget https://raw.github.com/ros-gbp/urdfdom-release/debian/hydro/precise/urdfdom/package.xml
```

### URDF Versioning

The URDF format supports versioning to allow for format evolution while maintaining compatibility. The version is specified in the `<robot>` tag using the `version` attribute in the format `x.y` (e.g., `"1.0"`, `"1.1"`).

#### Supported Versions

| Version | Status | Description |
|---------|--------|-------------|
| **1.0** | Default | The original URDF specification. If no version attribute is specified, version 1.0 is assumed. |
| **1.1** | Supported | Adds quaternion orientation support via the `quat_xyzw` attribute and capsule geometry. |

#### Version 1.0 (Default)

Version 1.0 is the default URDF version. When the `version` attribute is omitted from the `<robot>` tag, the parser assumes version 1.0.

Features supported in version 1.0:
- Robot model definition (links, joints, materials)
- Visual and collision geometry (box, cylinder, sphere, mesh)
- Joint types: revolute, continuous, prismatic, fixed, floating, planar
- Pose specification using `xyz` and `rpy` (roll-pitch-yaw) attributes
- Transmission elements
- Gazebo extensions and many more

Example:
```xml
<robot name="my_robot">
  <!-- No version attribute means version 1.0 -->
  <link name="base_link"/>
</robot>
```

Or explicitly:
```xml
<robot name="my_robot" version="1.0">
  <link name="base_link"/>
</robot>
```

#### Version 1.1

Version 1.1 extends the URDF specification with the following new features:
- All features from version 1.0
- **Quaternion orientation**: The `quat_xyzw` attribute can be used in `<origin>` elements as an alternative to `rpy` for specifying orientation
- **Capsule geometry**: A new geometry primitive defined by `radius` and `length` attributes

**Quaternion Example:**
```xml
<robot name="my_robot" version="1.1">
  <link name="base_link">
    <visual>
      <origin xyz="0 0 0" quat_xyzw="0 0 0 1"/>
      <geometry>
        <box size="1 1 1"/>
      </geometry>
    </visual>
  </link>
</robot>
```

**Note**: You cannot use both `rpy` and `quat_xyzw` in the same `<origin>` element.

**Capsule Geometry Example:**
```xml
<robot name="my_robot" version="1.1">
  <link name="arm_link">
    <visual>
      <geometry>
        <capsule radius="0.05" length="0.5"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <capsule radius="0.05" length="0.5"/>
      </geometry>
    </collision>
  </link>
</robot>
```

The capsule is a cylinder capped with hemispheres at both ends. The `length` attribute specifies the length of the cylindrical portion (not including the hemispherical caps), and `radius` specifies the radius of both the cylinder and the hemispheres. Both attributes must be non-negative finite values.

#### Compatibility and Parsing Behavior

| Scenario | Behavior |
|----------|----------|
| **Newer URDF parsed by older library** | Parsing will fail. The library validates the version and rejects URDF files with unsupported versions (e.g., a 1.1 URDF parsed by a library that only supports 1.0). Version-specific features like `quat_xyzw` and `capsule` geometry will be ignored with a warning if the declared version doesn't support them. |
| **Older URDF parsed by newer library** | Fully supported. The newer library is backward compatible and will parse older URDF files correctly. If no version attribute is specified, the parser defaults to version 1.0. |
| **Unsupported version (e.g., 2.0)** | Parsing will fail with an error. Only versions 1.0 and 1.1 are currently supported. |
| **Invalid version format** | Parsing will fail. The version must be in the format `x.y` (e.g., `"1.0"`). Single integers like `"1"` or malformed versions like `"1.0.0"` are rejected. |

**Version Format Requirements:**
- Must be in the form `major.minor` (e.g., `"1.0"`, `"1.1"`)
- Both major and minor must be non-negative integers
- Single integers (e.g., `"1"`) are not valid
- Trailing characters (e.g., `"1.0~pre6"`) are not allowed

### Installing from Source with ROS Debians

**Warning: this will break ABI compatibility with future /opt/ros updates through the debian package manager. This is a hack, use at your own risk.**

If you want to install urdfdom from source, but not install all of ROS from source, you can follow these loose guidelines.
This is not best practice for installing, but works.
This version is for ROS Hydro but should be easily customized for future version of ROS:

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
