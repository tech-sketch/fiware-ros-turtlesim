#!/usr/bin/env python
# -*- coding: utf-8 -*-
import unittest
from collections import namedtuple

from mock import patch
import freezegun

import rosunit
from std_msgs.msg import Float32

from fiware_ros_turtlesim.attribute_receiver import AttributeReceiver


class TestAttributeReceiver(unittest.TestCase):
    def setMock(self, mocked_rospy, mocked_mqtt):
        self.mocked_client = mocked_mqtt.Client.return_value

        mocked_rospy.get_param.return_value = {
            "mqtt": {
                "host": "testhost",
                "port": 1883,
                "topics": [{
                    "key": "attribute_receiver",
                    "name": "/mqtt/topics/attribute_receiver",
                    "format": "{timestamp}|temperature|{temperature}",
                }],
            },
            "ros": {
                "topics": [{
                    "key": "temperature",
                    "name": "/ros/topics/temperature",
                }],
            },
        }

    @patch('fiware_ros_turtlesim.attribute_receiver.mqtt')
    @patch('fiware_ros_turtlesim.attribute_receiver.rospy')
    def test_init(self, mocked_rospy, mocked_mqtt):
        node_name = 'foo'
        self.setMock(mocked_rospy, mocked_mqtt)

        receiver = AttributeReceiver(node_name)
        self.assertEqual(receiver.node_name, node_name)

        self.assertFalse(mocked_mqtt.called)
        mocked_mqtt.Client.assert_called_once_with(protocol=mocked_mqtt.MQTTv311)
        self.assertEqual(mocked_rospy.on_shutdown.call_count, 2)
        mocked_rospy.Subscriber.assert_called_once_with('/ros/topics/temperature', Float32, receiver._on_receive, queue_size=10)

    @patch('fiware_ros_turtlesim.attribute_receiver.mqtt')
    @patch('fiware_ros_turtlesim.attribute_receiver.rospy')
    def test_connect(self, mocked_rospy, mocked_mqtt):
        self.setMock(mocked_rospy, mocked_mqtt)

        AttributeReceiver('foo').connect()
        self.mocked_client.connect.assert_called_once_with('testhost', port=1883, keepalive=60)

    @patch('fiware_ros_turtlesim.attribute_receiver.mqtt')
    @patch('fiware_ros_turtlesim.attribute_receiver.rospy')
    def test_start(self, mocked_rospy, mocked_mqtt):
        self.setMock(mocked_rospy, mocked_mqtt)

        AttributeReceiver('foo').start()
        mocked_rospy.spin.assert_called_once_with()

    @freezegun.freeze_time('2018-01-02T03:04:05')
    @patch('fiware_ros_turtlesim.attribute_receiver.mqtt')
    @patch('fiware_ros_turtlesim.attribute_receiver.rospy')
    def test_on_receive(self, mocked_rospy, mocked_mqtt):
        self.setMock(mocked_rospy, mocked_mqtt)

        temperature = 1.25
        data_type = namedtuple('data', ('data',))
        AttributeReceiver('foo')._on_receive(data_type(data=temperature))
        message = '{}|temperature|{}'.format('2018-01-02T03:04:05.000000+0000', temperature)
        self.mocked_client.publish.assert_called_once_with('/mqtt/topics/attribute_receiver', message)


if __name__ == '__main__':
    rosunit.unitrun('fiware_ros_turtlesim', 'test_attribute_receiver', TestAttributeReceiver)
