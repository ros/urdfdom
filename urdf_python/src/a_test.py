#!/usr/bin/python

import roslib; roslib.load_manifest('urdf_python')
import rospy
import sys

from urdf_python.urdf import *

if __name__ == '__main__':
    for fn in sys.argv[1:]:
        try:
            model = URDF().load(fn)
            model.to_xml()
        except Exception as e:
            print "%s: "%fn, e
