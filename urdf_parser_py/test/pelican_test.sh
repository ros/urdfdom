#!/bin/bash
roslaunch ./pelican_test.launch
rosrun urdf_parser_py display_urdf -o /tmp/pelican_gen.urdf
python -c 'import rospy; open("/tmp/pelicant.urdf").write(rospy.get_param("/robot_description"))'
diff -u /tmp/pelican.urdf /tmp/pelican_gen.urdf