#! /usr/bin/env python2
## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=['knowledge_base'],
    package_dir={'': 'src'},
)

setup(**setup_args)
