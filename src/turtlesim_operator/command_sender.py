# -*- coding: utf-8 -*-
from math import pi

import rospy
from geometry_msgs.msg import Twist

import paho.mqtt.client as mqtt

HOST = 'localhost'
PORT = 1883
MQTT_TOPIC = '/ros/turtle/cmd'
ROS_TOPIC = '/turtle1/cmd_vel'
NODE_NAME = 'command_sender'

class CommandSender(object):
    def __init__(self):
        self.__client = mqtt.Client(protocol=mqtt.MQTTv311)
        self.__client.on_connect = self.__on_connect
        self.__client.on_message = self.__on_message

        rospy.on_shutdown(self.__client.disconnect)
        rospy.on_shutdown(self.__client.loop_stop)
        self.__ros_pub = rospy.Publisher(ROS_TOPIC, Twist, queue_size=10)

    def start(self):
        rospy.init_node(NODE_NAME)

        self.__client.connect(HOST, port=PORT, keepalive=60)
        self.__client.loop_start()

        rospy.spin()

    def __on_connect(self, client, userdata, flags, response_code):
        print('CommendSender.__on_connect: status={}'.format(response_code))
        client.subscribe(MQTT_TOPIC)

    def __on_message(self, client, userdata, msg):
        print('CommandSender.__on_message: msg={}'.format(str(msg.payload)))
        payload = str(msg.payload)
        if payload == 'circle':
            self.__do_circle()

    def __do_circle(self):
        print('do circle')

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

