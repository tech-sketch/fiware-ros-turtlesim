# -*- coding: utf-8 -*-
from collections import namedtuple

def __convert(obj):
    if isinstance(obj, dict):
        for key, value in obj.iteritems():
            obj[key] = __convert(value)
        return namedtuple('GenericDict', obj.keys())(**obj)
    elif isinstance(obj, list):
        return [__convert(item) for item in obj]
    else:
        return obj

def getParams(d):
    return __convert(d)
