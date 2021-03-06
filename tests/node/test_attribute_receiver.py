#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os
import time
import unittest

from mock import patch

import rostest
import rospy

from fiware_ros_turtlesim.attribute_receiver import AttributeReceiver

NODE_NAME = os.path.basename(__file__)


class TestAttributeReceiver(unittest.TestCase):

    @patch('fiware_ros_turtlesim.attribute_receiver.mqtt')
    def test_on_receive(self, mocked_mqtt):
        mocked_client = mocked_mqtt.Client.return_value

        AttributeReceiver(NODE_NAME).connect()

        self.assertFalse(mocked_mqtt.called)
        mocked_mqtt.Client.assert_called_once_with(protocol=mocked_mqtt.MQTTv311)
        mocked_client.connect.assert_called_once_with('testhost', port=1883, keepalive=60)

        count = 0
        while count < 60:
            if mocked_client.publish.called:
                self.assertTrue(True)
                return
            count += 1
            time.sleep(1)
        self.assertTrue(False)


if __name__ == '__main__':
    rospy.init_node(NODE_NAME, anonymous=True)
    rostest.rosrun('fiware_ros_turtlesim', 'test_attribute_receiver', TestAttributeReceiver)
