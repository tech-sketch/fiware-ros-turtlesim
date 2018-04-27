# -*- coding: utf-8 -*-
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
        self.__client.connect(self._params.mqtt.host, port=self._params.mqtt.port, keepalive=60)
        return self

    def start(self):
        logger.infof('Started Node : {}', self.node_name)

        rospy.spin()

    def _on_connect(self, client, userdata, flags, response_code):
        logger.infof('mqtt connect status={}', response_code)

    def _on_receive(self, data):
        logger.infof('received message from ros : {}', data.data)
        self.__client.publish(findItem(self._params.mqtt.topics, 'key', 'attribute_receiver').name, data.data)
