# -*- coding: utf-8 -*-
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

        rospy.on_shutdown(self.__client.disconnect)
        rospy.on_shutdown(self.__client.loop_stop)

        self._params = getParams(rospy.get_param("~"))
        topic = findItem(self._params.ros.topics, 'key', 'turtlesim')
        self.__ros_pub = rospy.Publisher(topic.name, Twist, queue_size=10)

    def connect(self):
        logger.infof('Connect mqtt broker')
        self.__client.connect(self._params.mqtt.host, port=self._params.mqtt.port, keepalive=60)
        self.__client.loop_start()
        return self

    def start(self):
        logger.infof('Started Node : {}', self.node_name)
        rospy.spin()

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
        logger.infof('received message from mqtt: {}', str(msg.payload))
        payload = str(msg.payload)
        if payload == 'circle':
            self._do_circle()

    def _do_circle(self):
        logger.infof('do circle')

        rate = 60
        r = rospy.Rate(rate)

        move_cmd = Twist()
        move_cmd.linear.x = 1.0
        move_cmd.angular.z = 1.0
        ticks = int(2 * pi * rate)
        for t in range(ticks + 1):
            self.__ros_pub.publish(move_cmd)
            r.sleep()
        self.__ros_pub.publish(Twist())
