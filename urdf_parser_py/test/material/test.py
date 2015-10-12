#!/usr/bin/python

import os
from urdf_parser_py import urdf
import subprocess as sp

orig = 'test.urdf'
gen = '/tmp/test.urdf'
patch = '/tmp/test.patch'

os.chdir(os.path.dirname(__file__))
robot = urdf.Robot.from_xml_file('test.urdf')

material = urdf.Material("Cyan", urdf.Color(0, 1.0, 1.0, 1.0))

material.check_valid()

robot.add_material(material)

for material in robot.materials:
  print type(material)

with open(gen, 'w') as f:
	f.write(robot.to_xml_string())
sp.call('diff -u "{}" "{}" > "{}"'.format(orig, gen, patch), shell=True)
