#!/usr/bin/env python
from math import pi

import rospy
from geometry_msgs.msg import Twist

import paho.mqtt.client as mqtt

HOST = 'localhost'
PORT = 1883
TOPIC = '/ros/turtle/cmd'

pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
rospy.init_node('command_sender', anonymous=True)

def on_connect(client, userdata, flags, response_code):
    print('status {}'.format(response_code))
    client.subscribe(TOPIC)

def on_message(client, userdata, msg):
    print('received msg from {}: {}'.format(msg.topic, str(msg.payload)))

    if str(msg.payload) != 'circle':
        return

    rate = 60
    r = rospy.Rate(rate)

    move_cmd = Twist()
    move_cmd.linear.x = 1.0
    move_cmd.angular.z = 1.0
    ticks = int(2 * pi * rate)
    for t in range(ticks + 1):
        pub.publish(move_cmd)
        r.sleep()
    pub.publish(Twist())

if __name__ == '__main__':
    try:
        c = mqtt.Client(protocol=mqtt.MQTTv311)
        c.on_connect = on_connect
        c.on_message = on_message
        c.connect(HOST, port=PORT, keepalive=60)
        c.loop_start()
        rospy.on_shutdown(c.disconnect)
        rospy.on_shutdown(c.loop_stop)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
