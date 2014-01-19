#!/usr/bin/python

import os
from urdf_parser_py import urdf
import subprocess as sp

orig = 'test.urdf'
gen = '/tmp/test.urdf'
patch = '/tmp/test.patch'

os.chdir(os.path.dirname(__file__))
robot = urdf.Robot.from_xml_file('test.urdf')

for transmission in robot.transmissions:
    print type(transmission)

with open(gen, 'w') as f:
	f.write(robot.to_xml_string())
sp.call('diff -u "{}" "{}" > "{}"'.format(orig, gen, patch), shell=True)
