# -*- coding: utf-8 -*-
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=['turtlesim_operator'],
    package_dir={'': 'src'},
    install_requires=['paho-mqtt',],
)

setup(**setup_args)
