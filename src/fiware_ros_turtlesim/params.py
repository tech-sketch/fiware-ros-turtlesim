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


def findItem(l, key_name, key_value):
    if not isinstance(l, list):
        return None
    if key_name is None or not isinstance(key_name, str):
        return None
    if key_value is None or not isinstance(key_value, str):
        return None
    return next((item for item in l if getattr(item, key_name, None) == key_value), None)
