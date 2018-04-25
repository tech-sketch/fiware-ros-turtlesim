#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy

from turtlesim_operator.command_sender import CommandSender

if __name__ == '__main__':
    try:
        sender = CommandSender()
        sender.start()
    except rospy.ROSInterruptException:
        pass
