#!/usr/bin/env python

from distutils.core import setup

d = {'author': u'Eric Cousineau',
 'author_email': 'eacousineau@gmail.com',
 'description': 'Reflection a la SDF from Gazebo.',
 'license': 'BSD',
 'maintainer': u'Eric Cousineau',
 'maintainer_email': 'eacousineau@gmail.com',
 'name': 'xml_reflection',
 'package_dir': {'': 'src'},
 'packages': ['xml_reflection'],
 'url': 'http://ros.org/wiki/xml_reflection',
 'version': '0.1.0'}

setup(**d)
