#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os
import unittest

import rostest
import rospy

from fiware_ros_turtlesim.command_sender import CommandSender

NODE_NAME = os.path.basename(__file__)


class TestCommandSender(unittest.TestCase):

    def test_params(self):
        sender = CommandSender(NODE_NAME)
        self.assertEqual(sender._params.mqtt.host, 'testhost')
        self.assertEqual(sender._params.mqtt.port, 1883)
        self.assertEqual(sender._params.mqtt.topics[0].key, 'command_sender')
        self.assertEqual(sender._params.mqtt.topics[0].name, '/mqtt/topics/command_sender')
        self.assertEqual(sender._params.mqtt.topics[0].re, '^(?P<device_id>.+)@move\\|(?P<cmd>.+)$')
        self.assertEqual(sender._params.mqtt.topics[1].key, 'command_sender_exec')
        self.assertEqual(sender._params.mqtt.topics[1].name, '/mqtt/topics/command_sender_exec')
        self.assertEqual(sender._params.mqtt.topics[1].format, '{device_id}@move|executed {cmd}')
        self.assertEqual(sender._params.mqtt.topics[2].key, 'attribute_receiver')
        self.assertEqual(sender._params.mqtt.topics[2].name, '/mqtt/topics/attribute_receiver')
        self.assertEqual(sender._params.mqtt.topics[2].format, '{timestamp}|temperature|{temperature}')
        self.assertEqual(sender._params.ros.rate, 60)
        self.assertEqual(sender._params.ros.topics[0].key, 'turtlesim')
        self.assertEqual(sender._params.ros.topics[0].name, '/ros/topics/turtlesim')
        self.assertEqual(sender._params.ros.topics[1].key, 'temperature')
        self.assertEqual(sender._params.ros.topics[1].name, '/ros/topics/temperature')


if __name__ == '__main__':
    rospy.init_node(NODE_NAME, anonymous=True)
    rostest.rosrun('fiware_ros_turtlesim', 'test_command_sender', TestCommandSender)
