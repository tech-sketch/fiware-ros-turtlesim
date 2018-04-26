# -*- coding: utf-8 -*-
from collections import namedtuple

GENERIC_DICT = 'GenericDict'

def __convert(obj):
    if isinstance(obj, dict):
        for key, value in obj.iteritems():
            obj[key] = __convert(value)
        return namedtuple(GENERIC_DICT, obj.keys())(**obj)
    elif isinstance(obj, list):
        return [__convert(item) for item in obj]
    else:
        return obj

def getParams(d):
    return __convert(d)
