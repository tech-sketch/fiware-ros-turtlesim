#!/usr/bin/env python
# -*- coding: utf-8 -*-
import unittest
from collections import namedtuple
from math import pi

from mock import MagicMock, patch, call

import rosunit
from geometry_msgs.msg import Twist

from fiware_ros_turtlesim.command_sender import CommandSender

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
                    "re": "^(?P<device_id>.+)@move\\|(?P<cmd>.+)$",
                },{
                    "key": "command_sender_exec",
                    "name": "/mqtt/topics/command_sender_exec",
                    "format": "{device_id}@move|executed {cmd}",
                }],
            },
            "ros": {
                "rate": 60,
                "topics": [{
                    "key": "turtlesim",
                    "name": "/ros/topics/turtlesim",
                }],
            },
        }

    @patch('fiware_ros_turtlesim.command_sender.mqtt')
    @patch('fiware_ros_turtlesim.command_sender.rospy')
    def test_init(self, mocked_rospy, mocked_mqtt):
        node_name = 'foo'
        self.setMock(mocked_rospy, mocked_mqtt)

        sender = CommandSender(node_name)
        self.assertEqual(sender.node_name, node_name)

        self.assertFalse(mocked_mqtt.called)
        mocked_mqtt.Client.assert_called_once_with(protocol=mocked_mqtt.MQTTv311)
        self.assertEqual(mocked_rospy.on_shutdown.call_count, 3)
        mocked_rospy.Publisher.assert_called_once_with('/ros/topics/turtlesim', Twist, queue_size=10)

    @patch('fiware_ros_turtlesim.command_sender.mqtt')
    @patch('fiware_ros_turtlesim.command_sender.rospy')
    def test_connect(self, mocked_rospy, mocked_mqtt):
        self.setMock(mocked_rospy, mocked_mqtt)

        CommandSender('foo').connect()
        self.mocked_client.connect.assert_called_once_with('testhost', port=1883, keepalive=60)
        self.mocked_client.loop_start.assert_called_once_with()

    @patch('fiware_ros_turtlesim.command_sender.mqtt')
    @patch('fiware_ros_turtlesim.command_sender.rospy')
    def test_start(self, mocked_rospy, mocked_mqtt):
        self.setMock(mocked_rospy, mocked_mqtt)

        CommandSender('foo').start()
        mocked_rospy.spin.assert_called_once_with()

    @patch('fiware_ros_turtlesim.command_sender.mqtt')
    @patch('fiware_ros_turtlesim.command_sender.rospy')
    def test_on_connect(self, mocked_rospy, mocked_mqtt):
        self.setMock(mocked_rospy, mocked_mqtt)

        CommandSender('foo')._on_connect(self.mocked_client, None, None, 0)
        self.mocked_client.subscribe.assert_called_once_with('/mqtt/topics/command_sender')

    @patch('fiware_ros_turtlesim.command_sender.mqtt')
    @patch('fiware_ros_turtlesim.command_sender.rospy')
    def test_on_message(self, mocked_rospy, mocked_mqtt):
        self.setMock(mocked_rospy, mocked_mqtt)

        msg_type = namedtuple('msg', ('payload',))

        sender = CommandSender('foo')

        sender._do_circle = MagicMock()
        sender._on_message(self.mocked_client, None, msg_type(payload='deviceid@move|circle'))
        sender._do_circle.assert_called_once_with()

        cmdexec = 'deviceid@move|executed circle'
        self.mocked_client.publish.assert_called_once_with('/mqtt/topics/command_sender_exec', cmdexec)

        sender._do_circle = MagicMock()
        sender._on_message(self.mocked_client, None, msg_type(payload='invalid'))
        sender._do_circle.assert_not_called()
        self.assertEqual(self.mocked_client.publish.call_count, 1)

    @patch('fiware_ros_turtlesim.command_sender.mqtt')
    @patch('fiware_ros_turtlesim.command_sender.rospy')
    def test_do_circle(self, mocked_rospy, mocked_mqtt):
        self.setMock(mocked_rospy, mocked_mqtt)

        mocked_pub = mocked_rospy.Publisher.return_value

        t = CommandSender('foo')._do_circle()
        t.join()
        mocked_rospy.Rate.assert_called_once_with(60)
        self.assertEqual(mocked_pub.publish.call_count, int(2 * pi * 60) + 1)
        args_list = mocked_pub.publish.call_args_list
        twist = Twist()
        twist.linear.x = 1.0
        twist.angular.z = 1.0
        for i in range(int(2 * pi * 60)):
            self.assertEqual(args_list[i], call(twist))
        self.assertEqual(args_list[-1], call(Twist()))

    @patch('fiware_ros_turtlesim.command_sender.mqtt')
    @patch('fiware_ros_turtlesim.command_sender.rospy')
    def test_do_square(self, mocked_rospy, mocked_mqtt):
        self.setMock(mocked_rospy, mocked_mqtt)

        mocked_pub = mocked_rospy.Publisher.return_value

        t = CommandSender('foo')._do_square()
        t.join()
        mocked_rospy.Rate.assert_called_with(60)
        self.assertEqual(mocked_pub.publish.call_count, 4 * (2 * 60 + 1) + 4 * (int(pi / 2 * 60) + 1))
        args_list = mocked_pub.publish.call_args_list
        linear = Twist()
        linear.linear.x = 1.0
        linear.angular.z = 0.0
        rotate = Twist()
        rotate.linear.x = 0.0
        rotate.angular.z = 1.0
        for i in range(2 * 60):
            self.assertEqual(args_list[i], call(linear))
        self.assertEqual(args_list[i + 1], call(Twist()))
        for j in range(int(pi / 2 * 60)):
            self.assertEqual(args_list[i + 2 + j], call(rotate))
        self.assertEqual(args_list[i + 2 + j + 1], call(Twist()))
        self.assertEqual(args_list[-1], call(Twist()))

    @patch('fiware_ros_turtlesim.command_sender.mqtt')
    @patch('fiware_ros_turtlesim.command_sender.rospy')
    def test_do_triangle(self, mocked_rospy, mocked_mqtt):
        self.setMock(mocked_rospy, mocked_mqtt)

        mocked_pub = mocked_rospy.Publisher.return_value

        t = CommandSender('foo')._do_triangle()
        t.join()
        mocked_rospy.Rate.assert_called_with(60)
        self.assertEqual(mocked_pub.publish.call_count, 3 * (2 * 60 + 1) + 3 * (int(pi * 2 / 3 * 60) + 1))
        args_list = mocked_pub.publish.call_args_list
        linear = Twist()
        linear.linear.x = 1.0
        linear.angular.z = 0.0
        rotate = Twist()
        rotate.linear.x = 0.0
        rotate.angular.z = 1.0
        for i in range(2 * 60):
            self.assertEqual(args_list[i], call(linear))
        self.assertEqual(args_list[i + 1], call(Twist()))
        for j in range(int(pi * 2 / 3 * 60)):
            self.assertEqual(args_list[i + 2 + j], call(rotate))
        self.assertEqual(args_list[i + 2 + j + 1], call(Twist()))
        self.assertEqual(args_list[-1], call(Twist()))

    @patch('fiware_ros_turtlesim.command_sender.mqtt')
    @patch('fiware_ros_turtlesim.command_sender.rospy')
    def test_do_forward(self, mocked_rospy, mocked_mqtt):
        self.setMock(mocked_rospy, mocked_mqtt)

        mocked_pub = mocked_rospy.Publisher.return_value

        t = CommandSender('foo')._do_forward()
        t.join()
        mocked_rospy.Rate.assert_called_with(60)
        self.assertEqual(mocked_pub.publish.call_count, int(0.2 * 60 + 1))
        args_list = mocked_pub.publish.call_args_list
        linear = Twist()
        linear.linear.x = 1.0
        linear.angular.z = 0.0
        for i in range(int(0.2 * 60)):
            self.assertEqual(args_list[i], call(linear))
        self.assertEqual(args_list[-1], call(Twist()))

    @patch('fiware_ros_turtlesim.command_sender.mqtt')
    @patch('fiware_ros_turtlesim.command_sender.rospy')
    def test_do_backward(self, mocked_rospy, mocked_mqtt):
        self.setMock(mocked_rospy, mocked_mqtt)

        mocked_pub = mocked_rospy.Publisher.return_value

        t = CommandSender('foo')._do_backward()
        t.join()
        mocked_rospy.Rate.assert_called_with(60)
        self.assertEqual(mocked_pub.publish.call_count, int(0.2 * 60 + 1))
        args_list = mocked_pub.publish.call_args_list
        linear = Twist()
        linear.linear.x = -1.0
        linear.angular.z = 0.0
        for i in range(int(0.2 * 60)):
            self.assertEqual(args_list[i], call(linear))
        self.assertEqual(args_list[-1], call(Twist()))

    @patch('fiware_ros_turtlesim.command_sender.mqtt')
    @patch('fiware_ros_turtlesim.command_sender.rospy')
    def test_do_turnleft(self, mocked_rospy, mocked_mqtt):
        self.setMock(mocked_rospy, mocked_mqtt)

        mocked_pub = mocked_rospy.Publisher.return_value

        t = CommandSender('foo')._do_turnleft()
        t.join()
        mocked_rospy.Rate.assert_called_with(60)
        self.assertEqual(mocked_pub.publish.call_count, (int(pi / 16 * 60) + 1))
        args_list = mocked_pub.publish.call_args_list
        rotate = Twist()
        rotate.linear.x = 0.0
        rotate.angular.z = 1.0
        for i in range(int(pi / 16 * 60)):
            self.assertEqual(args_list[i], call(rotate))
        self.assertEqual(args_list[-1], call(Twist()))

    @patch('fiware_ros_turtlesim.command_sender.mqtt')
    @patch('fiware_ros_turtlesim.command_sender.rospy')
    def test_do_turnright(self, mocked_rospy, mocked_mqtt):
        self.setMock(mocked_rospy, mocked_mqtt)

        mocked_pub = mocked_rospy.Publisher.return_value

        t = CommandSender('foo')._do_turnright()
        t.join()
        mocked_rospy.Rate.assert_called_with(60)
        self.assertEqual(mocked_pub.publish.call_count, (int(pi / 16 * 60) + 1))
        args_list = mocked_pub.publish.call_args_list
        rotate = Twist()
        rotate.linear.x = 0.0
        rotate.angular.z = -1.0
        for i in range(int(pi / 16 * 60)):
            self.assertEqual(args_list[i], call(rotate))
        self.assertEqual(args_list[-1], call(Twist()))


if __name__ == '__main__':
    rosunit.unitrun('fiware_ros_turtlesim', 'test_command_sender', TestCommandSender)
