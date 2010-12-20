#!/usr/bin/python
# -*- coding: utf-8 -*-
# 
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#     http://www.apache.org/licenses/LICENSE-2.0
# 
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License. 
import sys

if __name__=="__main__":
    if len(sys.argv) < 2:
        print "Usage:\n\tannotate_pr2dae.py [collada file]\n\nAnnoates the PR2 with the OpenRAVE profile tags.\nThis file will be in existence until URDF can support the extra information providedhere"
        sys.exit(0)

    dae = open(sys.argv[1],'r').read()
    if dae.find('<extra type="collision">') < 0:
        index = dae.find('</kinematics_model>')
        dae = dae[:index] + """
            <extra type="collision">
              <technique profile="OpenRAVE">
                <ignore_link_pair link0="kmodel0/base_link" link1="kmodel0/br_caster_l_wheel_link"/>
                <ignore_link_pair link0="kmodel0/base_link" link1="kmodel0/br_caster_r_wheel_link"/>
                <ignore_link_pair link0="kmodel0/base_link" link1="kmodel0/fr_caster_l_wheel_link"/>
                <ignore_link_pair link0="kmodel0/base_link" link1="kmodel0/fr_caster_r_wheel_link"/>
                <ignore_link_pair link0="kmodel0/base_link" link1="kmodel0/fl_caster_l_wheel_link"/>
                <ignore_link_pair link0="kmodel0/base_link" link1="kmodel0/fl_caster_r_wheel_link"/>
                <ignore_link_pair link0="kmodel0/base_link" link1="kmodel0/bl_caster_l_wheel_link"/>
                <ignore_link_pair link0="kmodel0/base_link" link1="kmodel0/bl_caster_r_wheel_link"/>
                <ignore_link_pair link0="kmodel0/base_link" link1="kmodel0/l_shoulder_pan_link"/>
                <ignore_link_pair link0="kmodel0/base_link" link1="kmodel0/r_shoulder_pan_link"/>
                <ignore_link_pair link0="kmodel0/torso_lift_link" link1="kmodel0/l_shoulder_lift_link"/>
                <ignore_link_pair link0="kmodel0/torso_lift_link" link1="kmodel0/r_shoulder_lift_link"/>
                <ignore_link_pair link0="kmodel0/l_shoulder_pan_link" link1="kmodel0/l_upper_arm_roll_link"/>
                <ignore_link_pair link0="kmodel0/r_shoulder_pan_link" link1="kmodel0/r_upper_arm_roll_link"/>
                <ignore_link_pair link0="kmodel0/l_elbow_flex_link" link1="kmodel0/l_forearm_link"/>
                <ignore_link_pair link0="kmodel0/r_elbow_flex_link" link1="kmodel0/r_forearm_link"/>
                <ignore_link_pair link0="kmodel0/l_upper_arm_link" link1="kmodel0/l_forearm_roll_link"/>
                <ignore_link_pair link0="kmodel0/r_upper_arm_link" link1="kmodel0/r_forearm_roll_link"/>
                <ignore_link_pair link0="kmodel0/l_forearm_link" link1="kmodel0/l_gripper_palm_link"/>
                <ignore_link_pair link0="kmodel0/r_forearm_link" link1="kmodel0/r_gripper_palm_link"/>
                <ignore_link_pair link0="kmodel0/l_forearm_link" link1="kmodel0/l_wrist_roll_link"/>
                <ignore_link_pair link0="kmodel0/r_forearm_link" link1="kmodel0/r_wrist_roll_link"/>
                <ignore_link_pair link0="kmodel0/l_gripper_l_finger_link" link1="kmodel0/l_gripper_r_finger_link"/>
                <ignore_link_pair link0="kmodel0/r_gripper_l_finger_link" link1="kmodel0/r_gripper_r_finger_link"/>
                <ignore_link_pair link0="kmodel0/l_gripper_l_finger_tip_link" link1="kmodel0/l_gripper_r_finger_link"/>
                <ignore_link_pair link0="kmodel0/r_gripper_l_finger_tip_link" link1="kmodel0/r_gripper_r_finger_link"/>
                <ignore_link_pair link0="kmodel0/l_gripper_l_finger_link" link1="kmodel0/l_gripper_r_finger_tip_link"/>
                <ignore_link_pair link0="kmodel0/r_gripper_l_finger_link" link1="kmodel0/r_gripper_r_finger_tip_link"/>
                <ignore_link_pair link0="kmodel0/torso_lift_link" link1="kmodel0/head_tilt_link"/>
              </technique>
            </extra>
""" + dae[index:]

    if dae.find('<extra type="library_sensors"') < 0:
        index = dae.find('<scene>')
        dae = dae[:index] + """
    <extra type="library_sensors" id="libsensors">
      <technique profile="OpenRAVE">
        <sensor type="base_laser2d" id="base_laser">
          <angle_range>-130 129.75</angle_range>
          <distance_range>0.023 60.0</distance_range>
          <angle_increment>0.25</angle_increment>
          <time_increment>1.73611115315e-05</time_increment>
          <measurement_time>0.05</measurement_time>
          <interface_type>
            <technique profile="OpenRAVE">
              <interface>BaseLaser2D</interface>
            </technique>
          </interface_type>
        </sensor>
        <sensor type="base_laser2d" id="tilt_laser">
          <angle_range>-90 89.75</angle_range>
          <distance_range>0.023 10.0</distance_range>
          <angle_increment>0.25</angle_increment>
          <time_increment>1.73611115315e-05</time_increment>
          <measurement_time>0.025</measurement_time>
          <interface_type>
            <technique profile="OpenRAVE">
              <interface>BaseLaser2D</interface>
            </technique>
          </interface_type>
        </sensor>
        <sensor type="base_pinhole_camera" id="high_def_sensor">
          <image_dimensions>2448 2050 3</image_dimensions>
          <format>uint8</format>
          <measurement_time>0.05</measurement_time>
          <intrinsic>2955 0 1224.5 0 2955 1025.5</intrinsic>
          <distortion>0 0 0 0 0</distortion>
          <interface_type>
            <technique profile="OpenRAVE">
              <interface>BaseCamera</interface>
            </technique>
          </interface_type>
        </sensor>
        <sensor type="base_pinhole_camera" id="wide_l_stereo_camera_sensor">
          <image_dimensions>640 480 1</image_dimensions>
          <format>uint8</format>
          <measurement_time>0.04</measurement_time>
          <intrinsic>395.71449999999999 0.0 335.86279000000002 0.0 395.71449999999999 245.62557000000001</intrinsic>
          <distortion>0 0 0 0 0</distortion>
          <interface_type>
            <technique profile="OpenRAVE">
              <interface>BaseCamera</interface>
            </technique>
          </interface_type>
        </sensor>
        <sensor type="base_pinhole_camera" id="wide_r_stereo_camera_sensor">
          <image_dimensions>640 480 1</image_dimensions>
          <format>uint8</format>
          <measurement_time>0.04</measurement_time>
          <intrinsic>395.71449999999999 0.0 335.86279000000002 0.0 395.71449999999999 245.62557000000001</intrinsic>
          <distortion>0 0 0 0 0</distortion>
          <interface_type>
            <technique profile="OpenRAVE">
              <interface>BaseCamera</interface>
            </technique>
          </interface_type>
        </sensor>
        <sensor type="base_stereo_camera" id="wide_stereo_camera_sensor">
          <instance_sensor url="#wide_l_stereo_camera_sensor">
            <rectification>0.99956000000000012 0.0027200000000000002 -0.029390000000000003 -0.0027700000000000003 0.99999000000000005 -0.0016800000000000001 0.029390000000000003 0.0017600000000000001 0.99957000000000007</rectification>
          </instance_sensor>
          <instance_sensor url="#wide_r_stereo_camera_sensor">
            <rectification>0.99941000000000013 0.0035200000000000001 -0.034260000000000006 -0.0034600000000000004 0.99999000000000005 0.0017800000000000001 0.034270000000000002 -0.0016600000000000002 0.99941000000000013</rectification>
          </instance_sensor>
        </sensor>
        <sensor type="base_pinhole_camera" id="l_forearm_cam_sensor">
          <image_dimensions>640 480 1</image_dimensions>
          <format>uint8</format>
          <measurement_time>0.04</measurement_time>
          <intrinsic>426.35142000000002 0.0 313.91464000000002 0.0 426.51092999999997 238.27394000000001</intrinsic>
          <distortion>0 0 0 0 0</distortion>
          <interface_type>
            <technique profile="OpenRAVE">
              <interface>BaseCamera</interface>
            </technique>
          </interface_type>
        </sensor>
        <sensor type="base_pinhole_camera" id="r_forearm_cam_sensor">
          <image_dimensions>640 480 1</image_dimensions>
          <format>uint8</format>
          <measurement_time>0.04</measurement_time>
          <intrinsic>430.5514 0.0 320.85068000000001 0.0 429.22170999999997 240.4314</intrinsic>
          <distortion>0 0 0 0 0</distortion>
          <interface_type>
            <technique profile="OpenRAVE">
              <interface>BaseCamera</interface>
            </technique>
          </interface_type>
        </sensor>
      </technique>
    </extra>
""" + dae[index:]

    manipulators = [('leftarm',"""
            <extra type="manipulator" name="leftarm">
              <technique profile="OpenRAVE">
                <frame_origin link="kmodel0/torso_lift_link"/>
                <frame_tip link="kmodel0/l_gripper_palm_link">
                  <translate>0.18 0 0</translate>
                  <rotate>0 1 0 90</rotate>
                </frame_tip>
                <gripper_axis axis="kmodel0/l_gripper_l_finger_joint/axis0">
                  <closingdirection>
                    <float>-1</float>
                  </closingdirection>
                </gripper_axis>
                <iksolver type="Transform6D">
                  <free_axis axis="kmodel0/l_upper_arm_roll_joint/axis0"/>
                  <interface_type>
                    <technique profile="OpenRAVE">
                      <interface>ikfast_pr2_leftarm</interface>
                    </technique>
                  </interface_type>
                </iksolver>
              </technique>
            </extra>
"""),
                    ('leftarm_torso',"""
            <extra type="manipulator" name="leftarm_torso">
              <technique profile="OpenRAVE">
                <frame_origin link="kmodel0/base_link"/>
                <frame_tip link="kmodel0/l_gripper_palm_link">
                  <translate>0.18 0 0</translate>
                  <rotate>0 1 0 90</rotate>
                </frame_tip>
                <gripper_axis axis="kmodel0/l_gripper_l_finger_joint/axis0">
                  <closingdirection>
                    <float>-1</float>
                  </closingdirection>
                </gripper_axis>
                <iksolver type="Transform6D">
                  <free_axis axis="kmodel0/torso_lift_joint/axis0"/>
                  <free_axis axis="kmodel0/l_upper_arm_roll_joint/axis0"/>
                  <interface_type>
                    <technique profile="OpenRAVE">
                      <interface>ikfast_pr2_leftarm_torso</interface>
                    </technique>
                  </interface_type>
                </iksolver>
              </technique>
            </extra>
"""),
                    ('rightarm',"""
            <extra type="manipulator" name="rightarm">
              <technique profile="OpenRAVE">
                <frame_origin link="kmodel0/torso_lift_link"/>
                <frame_tip link="kmodel0/r_gripper_palm_link">
                  <translate>0.18 0 0</translate>
                  <rotate>0 1 0 90</rotate>
                </frame_tip>
                <gripper_axis axis="kmodel0/r_gripper_l_finger_joint/axis0">
                  <closingdirection>
                    <float>-1</float>
                  </closingdirection>
                </gripper_axis>
                <iksolver type="Transform6D">
                  <free_axis axis="kmodel0/r_upper_arm_roll_joint/axis0"/>
                  <interface_type>
                    <technique profile="OpenRAVE">
                      <interface>ikfast_pr2_rightarm</interface>
                    </technique>
                  </interface_type>
                </iksolver>
              </technique>
            </extra>
"""),
                    ('rightarm_torso',"""
            <extra type="manipulator" name="rightarm_torso">
              <technique profile="OpenRAVE">
                <frame_origin link="kmodel0/base_link"/>
                <frame_tip link="kmodel0/r_gripper_palm_link">
                  <translate>0.18 0 0</translate>
                  <rotate>0 1 0 90</rotate>
                </frame_tip>
                <gripper_axis axis="kmodel0/r_gripper_l_finger_joint/axis0">
                  <closingdirection>
                    <float>-1</float>
                  </closingdirection>
                </gripper_axis>
                <iksolver type="Transform6D">
                  <free_axis axis="kmodel0/torso_lift_joint/axis0"/>
                  <free_axis axis="kmodel0/r_upper_arm_roll_joint/axis0"/>
                  <interface_type>
                    <technique profile="OpenRAVE">
                      <interface>ikfast_pr2_rightarm_torso</interface>
                    </technique>
                  </interface_type>
                </iksolver>
              </technique>
            </extra>
"""),
                    ('head',"""
            <extra type="manipulator" name="head">
              <technique profile="OpenRAVE">
                <frame_origin link="kmodel0/torso_lift_link"/>
                <frame_tip link="kmodel0/wide_stereo_optical_frame"/>
                <iksolver type="Lookat3D">
                  <interface_type>
                    <technique profile="OpenRAVE">
                      <interface>ikfast_pr2_head</interface>
                    </technique>
                  </interface_type>
                </iksolver>
              </technique>
            </extra>
"""),
                    ('head_torso',"""
            <extra type="manipulator" name="head_torso">
              <technique profile="OpenRAVE">
                <frame_origin link="kmodel0/base_link"/>
                <frame_tip link="kmodel0/wide_stereo_optical_frame"/>
                <iksolver type="Lookat3D">
                  <free_axis axis="kmodel0/torso_lift_joint/axis0"/>
                  <interface_type>
                    <technique profile="OpenRAVE">
                      <interface>ikfast_pr2_head_torso</interface>
                    </technique>
                  </interface_type>
                </iksolver>
              </technique>
            </extra>
"""),
                    ('rightarm_camera',"""
            <extra type="manipulator" name="rightarm_camera">
              <technique profile="OpenRAVE">
                <frame_origin link="kmodel0/torso_lift_link"/>
                <frame_tip link="kmodel0/r_forearm_cam_optical_frame">
                </frame_tip>
                <iksolver type="Ray4D">
                </iksolver>
              </technique>
            </extra>
"""),
                    ('leftarm_camera',"""
            <extra type="manipulator" name="leftarm_camera">
              <technique profile="OpenRAVE">
                <frame_origin link="kmodel0/torso_lift_link"/>
                <frame_tip link="kmodel0/l_forearm_cam_optical_frame">
                </frame_tip>
                <iksolver type="Ray4D">
                </iksolver>
              </technique>
            </extra>
""")]
    sensors = [('base_laser',"""
            <extra type="attach_sensor" name="base_laser">
              <technique profile="OpenRAVE">
                <instance_sensor url="#base_laser"/>
                <frame_origin link="kmodel0/base_laser_link"/>
              </technique>
            </extra>
"""),
               ('tilt_laser',"""
            <extra type="attach_sensor" name="tilt_laser">
              <technique profile="OpenRAVE">
                <instance_sensor url="#tilt_laser"/>
                <frame_origin link="kmodel0/laser_tilt_link"/>
              </technique>
            </extra>
"""),
               ('l_forearm_cam_optical_sensor',"""
            <extra type="attach_sensor" name="l_forearm_cam_optical_sensor">
              <technique profile="OpenRAVE">
                <instance_sensor url="#l_forearm_cam_sensor"/>
                <frame_origin link="kmodel0/l_forearm_cam_optical_frame"/>
              </technique>
            </extra>
"""),
               ('r_forearm_cam_optical_sensor',"""
            <extra type="attach_sensor" name="r_forearm_cam_optical_sensor">
              <technique profile="OpenRAVE">
                <instance_sensor url="#l_forearm_cam_sensor"/>
                <frame_origin link="kmodel0/r_forearm_cam_optical_frame"/>
              </technique>
            </extra>
"""),
               ('narrow_stereo_optical_sensor',"""
            <extra type="attach_sensor" name="narrow_stereo_optical_sensor">
              <technique profile="OpenRAVE">
                <instance_sensor url="#narrow_stereo_camera_sensor"/>
                <frame_origin link="kmodel0/narrow_stereo_optical_frame"/>
              </technique>
            </extra>
"""),
               ('wide_stereo_optical_sensor',"""
            <extra type="attach_sensor" name="wide_stereo_optical_sensor">
              <technique profile="OpenRAVE">
                <instance_sensor url="#wide_stereo_camera_sensor"/>
                <frame_origin link="kmodel0/wide_stereo_optical_frame"/>
              </technique>
            </extra>
""")]
    for name,xml in manipulators:
        if dae.find('<extra type="manipulator" name="%s">'%name) < 0:
            index = dae.find('</motion>')+9
            dae = dae[:index] + xml + dae[index:]
    for name,xml in sensors:
        if dae.find('<extra type="attach_sensor" name="%s">'%name) < 0:
            index = dae.find('</motion>')+9
            dae = dae[:index] + xml + dae[index:]
    open(sys.argv[1],'w').write(dae)
