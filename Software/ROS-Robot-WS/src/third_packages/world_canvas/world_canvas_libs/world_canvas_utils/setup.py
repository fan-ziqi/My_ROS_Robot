#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['world_canvas_utils'],
    package_dir={'': 'src'},
    requires=['roslib', 'rospy', 'world_canvas_msgs']
)

setup(**d)
