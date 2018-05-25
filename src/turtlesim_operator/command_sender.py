# -*- coding: utf-8 -*-
import os
import re
import ssl
import threading
from math import pi

import rospy
from geometry_msgs.msg import Twist

import paho.mqtt.client as mqtt

from turtlesim_operator.params import getParams, findItem
from turtlesim_operator.logging import getLogger
logger = getLogger(__name__)


class CommandSender(object):
    def __init__(self, node_name):
        self.node_name = node_name
        self.__client = mqtt.Client(protocol=mqtt.MQTTv311)
        self.__client.on_connect = self._on_connect
        self.__client.on_message = self._on_message

        rospy.on_shutdown(self._do_stop)
        rospy.on_shutdown(self.__client.disconnect)
        rospy.on_shutdown(self.__client.loop_stop)

        self._params = getParams(rospy.get_param("~"))
        topic = findItem(self._params.ros.topics, 'key', 'turtlesim')
        self.__ros_pub = rospy.Publisher(topic.name, Twist, queue_size=10)
        self.__do_move = False
        self.__lock = threading.Lock()

        self._cmd_payload_re = re.compile(findItem(self._params.mqtt.topics, 'key', 'command_sender').re)

    def connect(self):
        logger.infof('Connect mqtt broker')

        if hasattr(self._params.mqtt, 'cafile'):
            cafile_path = self._params.mqtt.cafile.strip()
            if len(cafile_path) > 0 and os.path.isfile(cafile_path):
                self.__client.tls_set(cafile_path, tls_version=ssl.PROTOCOL_TLSv1_2)

        if hasattr(self._params.mqtt, 'username') and hasattr(self._params.mqtt, 'password'):
            username = self._params.mqtt.username.strip()
            password = self._params.mqtt.password.strip()
            if len(username) > 0 and len(password) > 0:
                self.__client.username_pw_set(username, password)

        self.__client.connect(self._params.mqtt.host, port=self._params.mqtt.port, keepalive=60)
        self.__client.loop_start()
        return self

    def start(self):
        logger.infof('CommandSender start : {}', self.node_name)
        rospy.spin()
        logger.infof('CommandSender stop : {}', self.node_name)

    def nodetest(self):
        from collections import namedtuple
        logger.warnf('Test publish using publishtest of rostest')
        r = rospy.Rate(0.5)
        while not rospy.is_shutdown():
            self._on_message(None, None, namedtuple('msg', ('payload',))(payload='circle'))
            r.sleep()

    def _on_connect(self, client, userdata, flags, response_code):
        logger.infof('mqtt connect status={}', response_code)
        client.subscribe(findItem(self._params.mqtt.topics, 'key', 'command_sender').name)

    def _on_message(self, client, userdata, msg):
        payload = str(msg.payload)
        logger.infof('received message from mqtt: {}', payload)
        matcher = self._cmd_payload_re.match(payload)
        if matcher:
            cmd = matcher.group('cmd')
            device_id = matcher.group('device_id')
            cmdexe = 'MOVED: {}'.format(cmd)
            if cmd == 'circle':
                self._do_circle()
            elif cmd == 'square':
                self._do_square()
            elif cmd == 'triangle':
                self._do_triangle()
            elif cmd == 'cross':
                self._do_stop()
            elif cmd == 'up':
                self._do_forward()
            elif cmd == 'down':
                self._do_backward()
            elif cmd == 'left':
                self._do_turnleft()
            elif cmd == 'right':
                self._do_turnright()
            else:
                logger.warnf('unknown cmd: {}', payload)
                cmd = 'UNKNOWN CMD: {}'.format(cmd)
            topic = findItem(self._params.mqtt.topics, 'key', 'command_sender_exec').name
            fmt = findItem(self._params.mqtt.topics, 'key', 'command_sender_exec').format
            self.__client.publish(topic, fmt.format(device_id=device_id, cmd=cmd))
        else:
            logger.warnf('unkown payload: {}', payload)
        logger.debugf('active threds = {}', threading.active_count())

    def _do_circle(self):
        logger.infof('do circle')
        def move(self):
            self.__circle(int(2 * pi * self._params.ros.rate))
        return self._do_move(move)

    def _do_square(self):
        logger.infof('do square')
        def move(self):
            self.__linear(2 * self._params.ros.rate)
            self.__rotate(pi / 2)
            self.__linear(2 * self._params.ros.rate)
            self.__rotate(pi / 2)
            self.__linear(2 * self._params.ros.rate)
            self.__rotate(pi / 2)
            self.__linear(2 * self._params.ros.rate)
            self.__rotate(pi / 2)
        return self._do_move(move)

    def _do_triangle(self):
        logger.infof('do triangle')
        def move(self):
            self.__linear(2 * self._params.ros.rate)
            self.__rotate(pi * 2 / 3)
            self.__linear(2 * self._params.ros.rate)
            self.__rotate(pi * 2 / 3)
            self.__linear(2 * self._params.ros.rate)
            self.__rotate(pi * 2 / 3)
        return self._do_move(move)

    def _do_forward(self):
        logger.infof('do forward')
        def move(self):
            self.__linear(int(self._params.ros.rate * 0.2))
        return self._do_move(move)

    def _do_backward(self):
        logger.infof('do backward')
        def move(self):
            self.__linear(int(self._params.ros.rate * 0.2), reverse=True)
        return self._do_move(move)

    def _do_turnleft(self):
        logger.infof('do turn left')
        def move(self):
            self.__rotate(pi / 16)
        return self._do_move(move)

    def _do_turnright(self):
        logger.infof('do turn right')
        def move(self):
            self.__rotate(pi / 16, reverse=True)
        return self._do_move(move)

    def _do_stop(self):
        with self.__lock:
            self.__do_move = False
        logger.infof('sotp moving')

    def _do_move(self, callback):
        def func():
            if not callable(callback):
                return

            if self.__do_move:
                logger.infof('now moving')
                return

            with self.__lock:
                self.__do_move = True

            callback(self)

            with self.__lock:
                self.__do_move = False
        thread = threading.Thread(target=func)
        thread.start()
        return thread

    def __circle(self, ticks):
        move_cmd = Twist()
        move_cmd.linear.x = 1.0
        move_cmd.angular.z = 1.0
        self.__move(ticks, move_cmd)

    def __linear(self, ticks, reverse=False):
        move_cmd = Twist()
        move_cmd.linear.x = 1.0 if not reverse else -1.0
        self.__move(ticks, move_cmd)

    def __rotate(self, angle, reverse=False):
        move_cmd = Twist()
        move_cmd.angular.z = 1.0 if not reverse else -1.0
        ticks = int(angle * self._params.ros.rate)
        self.__move(ticks, move_cmd)

    def __move(self, ticks, move_cmd):
        r = rospy.Rate(self._params.ros.rate)
        for t in range(ticks):
            if not self.__do_move:
                break
            self.__ros_pub.publish(move_cmd)
            r.sleep()
        self.__ros_pub.publish(Twist())
