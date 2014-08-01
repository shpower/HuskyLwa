# Software License Agreement (proprietary) 
#
# @author    Arnold <akalmbach@clearpathrobotics.com>
# @copyright (c) 2014, Clearpath Robotics, Inc., All rights reserved.
#
# Redistribution and use in source and binary forms, with or without modification, is not permitted without the
# express permission of Clearpath Robotics.
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
            packages=['robotiq_s_model_control'],
                package_dir={'': 'src'},
                    requires=['message_generation', 'message_runtime', 'rospy']
                    )

setup(**setup_args)
