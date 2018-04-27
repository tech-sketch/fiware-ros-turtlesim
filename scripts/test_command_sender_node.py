#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os

import rospy
from turtlesim_operator.command_sender import CommandSender
from turtlesim_operator.logging import getLogger
logger = getLogger(__name__)

NODE_NAME = os.path.basename(__file__)

def main():
    try:
        rospy.init_node(NODE_NAME)
        logger.infof('Start node : {}', NODE_NAME)
        sender = CommandSender(NODE_NAME)
        sender.connect().start()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
