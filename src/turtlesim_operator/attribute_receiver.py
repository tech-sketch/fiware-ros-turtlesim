# -*- coding: utf-8 -*-
import os
import ssl
from datetime import datetime

import pytz

import rospy
from std_msgs.msg import Float32

import paho.mqtt.client as mqtt

from turtlesim_operator.params import getParams, findItem
from turtlesim_operator.logging import getLogger
logger = getLogger(__name__)


class AttributeReceiver(object):
    def __init__(self, node_name):
        self.node_name = node_name
        self.__client = mqtt.Client(protocol=mqtt.MQTTv311)
        self.__client.on_connect = self._on_connect

        rospy.on_shutdown(self.__client.disconnect)
        rospy.on_shutdown(self.__client.loop_stop)

        self._params = getParams(rospy.get_param("~"))
        topic = findItem(self._params.ros.topics, 'key', 'temperature')
        self.__ros_sub = rospy.Subscriber(topic.name, Float32, self._on_receive, queue_size=10)

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
        logger.infof('AttributeReceiver start : {}', self.node_name)
        rospy.spin()
        logger.infof('AttributeReceiver stop : {}', self.node_name)

    def _on_connect(self, client, userdata, flags, response_code):
        logger.infof('mqtt connect status={}', response_code)

    def _on_receive(self, data):
        logger.infof('received message from ros : {}', data.data)
        now = datetime.now(pytz.utc).strftime('%Y-%m-%dT%H:%M:%S.%f%z')
        topic = findItem(self._params.mqtt.topics, 'key', 'attribute_receiver').name
        fmt = findItem(self._params.mqtt.topics, 'key', 'attribute_receiver').format
        msg = fmt.format(timestamp=now, temperature=data.data)
        self.__client.publish(topic, msg)
