#!/usr/bin/env python
# -*- coding: utf-8 -*-
import unittest
from collections import namedtuple
from math import pi

from mock import MagicMock, patch, call

import rosunit
from geometry_msgs.msg import Twist

from turtlesim_operator.command_sender import CommandSender

class TestCommandSender(unittest.TestCase):
    def setMock(self, mocked_rospy, mocked_mqtt):
        self.mocked_client = mocked_mqtt.Client.return_value

        mocked_rospy.get_param.return_value = {
            "mqtt": {
                "host": "testhost",
                "port": 1883,
                "topics": [{
                    "key": "command_sender",
                    "name": "/mqtt/topics/command_sender",
                }],
            },
            "ros": {
                "topics": [{
                    "key": "turtlesim",
                    "name": "/ros/topics/turtlesim",
                }],
            },
        }

    @patch('turtlesim_operator.command_sender.mqtt')
    @patch('turtlesim_operator.command_sender.rospy')
    def test_init(self, mocked_rospy, mocked_mqtt):
        node_name = 'foo'
        self.setMock(mocked_rospy, mocked_mqtt)

        sender = CommandSender(node_name)
        self.assertEqual(sender.node_name, node_name)

        self.assertFalse(mocked_mqtt.called)
        mocked_mqtt.Client.assert_called_once_with(protocol=mocked_mqtt.MQTTv311)
        self.assertEqual(mocked_rospy.on_shutdown.call_count, 2)
        mocked_rospy.Publisher.assert_called_once_with('/ros/topics/turtlesim', Twist, queue_size=10)

    @patch('turtlesim_operator.command_sender.mqtt')
    @patch('turtlesim_operator.command_sender.rospy')
    def test_connect(self, mocked_rospy, mocked_mqtt):
        self.setMock(mocked_rospy, mocked_mqtt)

        CommandSender('foo').connect()
        self.mocked_client.connect.assert_called_once_with('testhost', port=1883, keepalive=60)
        self.mocked_client.loop_start.assert_called_once_with()

    @patch('turtlesim_operator.command_sender.mqtt')
    @patch('turtlesim_operator.command_sender.rospy')
    def test_start(self, mocked_rospy, mocked_mqtt):
        self.setMock(mocked_rospy, mocked_mqtt)

        CommandSender('foo').start()
        mocked_rospy.spin.assert_called_once_with()

    @patch('turtlesim_operator.command_sender.mqtt')
    @patch('turtlesim_operator.command_sender.rospy')
    def test_on_connect(self, mocked_rospy, mocked_mqtt):
        self.setMock(mocked_rospy, mocked_mqtt)

        CommandSender('foo')._on_connect(self.mocked_client, None, None, 0)
        self.mocked_client.subscribe.assert_called_once_with('/mqtt/topics/command_sender')

    @patch('turtlesim_operator.command_sender.mqtt')
    @patch('turtlesim_operator.command_sender.rospy')
    def test_on_message(self, mocked_rospy, mocked_mqtt):
        self.setMock(mocked_rospy, mocked_mqtt)

        msg_type = namedtuple('msg', ('payload',))

        sender = CommandSender('foo')

        sender._do_circle = MagicMock()
        sender._on_message(self.mocked_client, None, msg_type(payload='circle'))
        sender._do_circle.assert_called_once_with()

        sender._do_circle = MagicMock()
        sender._on_message(self.mocked_client, None, msg_type(payload='invalid'))
        sender._do_circle.assert_not_called()

    @patch('turtlesim_operator.command_sender.mqtt')
    @patch('turtlesim_operator.command_sender.rospy')
    def test_do_circle(self, mocked_rospy, mocked_mqtt):
        self.setMock(mocked_rospy, mocked_mqtt)

        mocked_pub = mocked_rospy.Publisher.return_value

        CommandSender('foo')._do_circle()
        mocked_rospy.Rate.assert_called_once_with(60)
        self.assertEqual(mocked_pub.publish.call_count, int(2 * pi * 60) + 2)
        args_list = mocked_pub.publish.call_args_list
        twist = Twist()
        twist.linear.x = 1.0
        twist.angular.z = 1.0
        for i in range(int(2 * pi * 60) + 1):
            self.assertEqual(args_list[i], call(twist))
        self.assertEqual(args_list[-1], call(Twist()))

if __name__ == '__main__':
    rosunit.unitrun('turtlesim_operator', 'test_command_sender', TestCommandSender)
