# -*- coding: utf-8 -*-
import inspect

import rospy


class __Logger(object):
    def __init__(self, name):
        self.name = name

    def __log(self, func, msg, *args, **kwargs):
        stack = inspect.stack()
        caller = []
        try:
            caller.append(stack[2][0].f_locals['self'].__class__.__name__)
        except KeyError:
            pass
        caller.append(stack[2][0].f_code.co_name)

        try:
            getattr(rospy, func)('[{}:{}] {}'.format(self.name, '.'.join(caller), msg.format(*args, **kwargs)))
        except AttributeError:
            pass

    def debugf(self, msg, *args, **kwargs):
        self.__log('logdebug', msg, *args, **kwargs)

    def infof(self, msg, *args, **kwargs):
        self.__log('loginfo', msg, *args, **kwargs)

    def warnf(self, msg, *args, **kwargs):
        self.__log('logwarn', msg, *args, **kwargs)

    def errf(self, msg, *args, **kwargs):
        self.__log('logerr', msg, *args, **kwargs)

    def fatalf(self, msg, *args, **kwargs):
        self.__log('logfatal', msg, *args, **kwargs)


def getLogger(name):
    return __Logger(name)
