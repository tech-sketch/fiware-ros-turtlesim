#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os

import rospy
from std_msgs.msg import Float32

from turtlesim_operator.params import getParams, findItem
from turtlesim_operator.logging import getLogger
logger = getLogger(__name__)

NODE_NAME = os.path.basename(__file__)

def main():
    try:
        rospy.init_node(NODE_NAME)
        logger.warnf('Start test node : {}', NODE_NAME)
        topic = findItem(getParams(rospy.get_param("~")).ros.topics, 'key', 'temperature')
        pub = rospy.Publisher(topic.name, Float32, queue_size=10)
        r = rospy.Rate(2)
        while not rospy.is_shutdown():
            pub.publish(Float32(1.0))
            r.sleep()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
