#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os
import unittest
from collections import namedtuple

import rostest
import rospy

from turtlesim_operator.command_sender import CommandSender
from turtlesim_operator.params import GENERIC_DICT

NODE_NAME = os.path.basename(__file__)

class TestCommandSender(unittest.TestCase):

    def test_params(self):
        base_type = namedtuple(GENERIC_DICT, ('mqtt', 'ros', 'mode'))
        mqtt_type = namedtuple(GENERIC_DICT, ('host', 'port', 'topics'))
        ros_type = namedtuple(GENERIC_DICT, ('topics',))
        topics_type = namedtuple(GENERIC_DICT, ('name', 'key'))

        expect = base_type(mqtt=mqtt_type(host='testhost',
                                          port=1883,
                                          topics=[topics_type(key='command_sender', name='/mqtt/topics/command_sender'),
                                                  topics_type(key='attribute_receiver', name='/mqtt/topics/attribute_receiver'),]),
                           ros=ros_type(topics=[topics_type(key='turtlesim', name='/ros/topics/turtlesim'),
                                                topics_type(key='temperature', name='/ros/topics/temperature'),]),
                           mode='nodetest',)
        sender = CommandSender(NODE_NAME)
        self.assertEqual(sender._params, expect)

if __name__ == '__main__':
    rospy.init_node(NODE_NAME, anonymous=True)
    rostest.rosrun('turtlesim_operator', 'test_command_sender', TestCommandSender)
