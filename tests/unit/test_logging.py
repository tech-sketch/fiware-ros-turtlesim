#!/usr/bin/env python
# -*- coding: utf-8 -*-
import unittest
import sys

from mock import patch

from ddt import ddt, data, unpack

import rosunit

from fiware_ros_turtlesim.logging import getLogger


@ddt
class TestGetLogger(unittest.TestCase):

    @data({'logm': 'debugf', 'rosm': 'logdebug'},
          {'logm': 'infof', 'rosm': 'loginfo'},
          {'logm': 'warnf', 'rosm': 'logwarn'},
          {'logm': 'errf', 'rosm': 'logerr'},
          {'logm': 'fatalf', 'rosm': 'logfatal'},)
    @unpack
    @patch('fiware_ros_turtlesim.logging.rospy')
    def test_logger_wo_params(self, mocked_rospy, logm, rosm):
        name = 'foo'
        message = 'test message'
        log_message = '[{name}:{caller}] {message}'.format(
            name=name,
            caller=self.__class__.__name__ + '.' + sys._getframe().f_code.co_name,
            message=message,
        )

        logger = getLogger(name)
        self.assertEqual(logger.name, name)

        getattr(logger, logm)(message)
        self.assertFalse(mocked_rospy.called)
        self.assertTrue(getattr(mocked_rospy, rosm).called)
        getattr(mocked_rospy, rosm).assert_called_once_with(log_message)

    @data({'logm': 'debugf', 'rosm': 'logdebug'},
          {'logm': 'infof', 'rosm': 'loginfo'},
          {'logm': 'warnf', 'rosm': 'logwarn'},
          {'logm': 'errf', 'rosm': 'logerr'},
          {'logm': 'fatalf', 'rosm': 'logfatal'},)
    @unpack
    @patch('fiware_ros_turtlesim.logging.rospy')
    def test_logger_w_args(self, mocked_rospy, logm, rosm):
        name = 'foo'
        message = 'test message'
        arg0 = 'arg0'
        arg1 = 'arg1'
        kwargs0 = 'kwargs0'
        kwargs1 = 'kwargs1'
        log_message = '[{name}:{caller}] {message}, {arg1}, {kwargs0}, {arg0}, {kwargs1}'.format(
            name=name,
            caller=self.__class__.__name__ + '.' + sys._getframe().f_code.co_name,
            message=message,
            arg0=arg0,
            arg1=arg1,
            kwargs0=kwargs0,
            kwargs1=kwargs1,
        )

        logger = getLogger(name)
        self.assertEqual(logger.name, name)

        getattr(logger, logm)(message + ', {1}, {kwargs0}, {0}, {kwargs1}', arg0, arg1, kwargs1=kwargs1, kwargs0=kwargs0)
        self.assertFalse(mocked_rospy.called)
        self.assertTrue(getattr(mocked_rospy, rosm).called)
        getattr(mocked_rospy, rosm).assert_called_once_with(log_message)


if __name__ == '__main__':
    rosunit.unitrun('fiware_ros_turtlesim', 'test_logging', TestGetLogger)
