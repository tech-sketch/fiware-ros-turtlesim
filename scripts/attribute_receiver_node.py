#!/usr/bin/python
# -*- coding: utf-8 -*-
import os

import rospy

from fiware_ros_turtlesim.attribute_receiver import AttributeReceiver
from fiware_ros_turtlesim.logging import getLogger
logger = getLogger(__name__)

NODE_NAME = os.path.basename(__file__)


def main():
    try:
        rospy.init_node(NODE_NAME)
        logger.infof('Start node : {}', NODE_NAME)
        receiver = AttributeReceiver(NODE_NAME)
        receiver.connect().start()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
