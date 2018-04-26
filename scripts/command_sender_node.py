#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy

from turtlesim_operator.command_sender import CommandSender
from turtlesim_operator.logging import getLogger
logger = getLogger(__name__)

NODE_NAME = 'command_sender_node'
LOG_LEVEL = rospy.DEBUG

def main():
    try:
        rospy.init_node(NODE_NAME, log_level=LOG_LEVEL)
        logger.infof('Start Node : {}', NODE_NAME)
        sender = CommandSender()
        sender.start()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
