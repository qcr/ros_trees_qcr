#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

import os

def get_packages(dir):
  if not '__init__.py' in os.listdir(dir):
    return []

  subdirectories = filter(lambda x: os.path.isdir(os.path.join(dir, x)), os.listdir(dir))

  result = []
  
  for sd in subdirectories:
    result += get_packages(os.path.join(dir, sd))

  result.append(dir.replace('src/', '').replace('/', '.'))
  return result

d = generate_distutils_setup(
  packages=get_packages('src/ros_leaves'), 
  package_dir={'': 'src'}
)

setup(**d)
