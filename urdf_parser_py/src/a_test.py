#!/usr/bin/python

import roslib; roslib.load_manifest('urdf_parser_py')
import rospy
import sys

from urdf_parser_py.urdf import *

if __name__ == '__main__':
    for fn in sys.argv[1:]:
        rospy.loginfo(fn)
        model = URDF().load(fn)
        model.to_xml()

