#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os
import unittest
from collections import namedtuple

from mock import MagicMock, patch

import rostest
import rospy

from turtlesim_operator.command_sender import CommandSender

NODE_NAME = os.path.basename(__file__)

class TestTurtlesimOperator(unittest.TestCase):
    def setUp(self):
        super(TestTurtlesimOperator, self).setUp()
        with patch('turtlesim_operator.command_sender.mqtt') as mocked_mqtt:
            self.mocked_client = MagicMock()
            mocked_mqtt.Client = self.mocked_client

            self.sender = CommandSender()

            self.assertTrue(mocked_mqtt.Client.called)
            self.assertTrue(self.mocked_client.called)
            self.mocked_client.assert_called_once_with(protocol=mocked_mqtt.MQTTv311)

    def test_start(self):
        self.assertTrue(True)

if __name__ == '__main__':
    rospy.init_node(NODE_NAME)
    rostest.rosrun('turtlesim_operator', 'test_turtlesim_operator', TestTurtlesimOperator)
