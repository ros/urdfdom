#!/usr/bin/env python

from distutils.core import setup

d = {'author': u'Thomas Moulard <thomas.moulard@gmail.com>, David Lu <davidlu@wustl.edu>, Kelsey Hawkins <kphawkins@gmail.com>, Antonio El Khoury <aelkhour@laas.fr>, Eric Cousineau <eacousineau@gmail.com>',
 'description': 'The urdf_parser_py package contains a Python implementation of the\nurdf_parser modeling various aspects of robot information, specified in the\nXml Robot Description Format (URDF).',
 'license': 'BSD',
 'maintainer': u'Thomas Moulard',
 'maintainer_email': 'thomas.moulard@gmail.com',
 'name': 'urdf_parser_py',
 'package_dir': {'': 'src'},
 'packages': ['urdf_parser_py'],
 'url': 'http://ros.org/wiki/urdf_parser_py',
 'version': '0.3.0'}

setup(**d)
