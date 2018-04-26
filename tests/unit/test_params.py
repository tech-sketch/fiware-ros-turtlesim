#!/usr/bin/env python
# -*- coding: utf-8 -*-
import unittest
from collections import namedtuple

import rosunit

from turtlesim_operator.params import getParams, GENERIC_DICT

class TestParams(unittest.TestCase):
    def test_empty_dict(self):
        result = getParams({})
        self.assertEqual(result, namedtuple(GENERIC_DICT, ())())

        with self.assertRaises(AttributeError):
            result.foo

    def test_simple_dict(self):
        result = getParams({'a': 'A'})
        self.assertEqual(result, namedtuple(GENERIC_DICT, ('a'))(a='A'))
        self.assertEqual(result.a, 'A')

    def test_nested_dict(self):
        result = getParams({'a': {'aa': 'aA', 'ab': {'aba': 'abA'}}, 'b': 'B'})
        ab = namedtuple(GENERIC_DICT, ('aba'))(aba='abA')
        a = namedtuple(GENERIC_DICT, ('aa', 'ab'))(aa='aA', ab=ab)
        self.assertEqual(result, namedtuple(GENERIC_DICT, ('a', 'b'))(a=a, b='B'))
        self.assertEqual(result.a.aa, 'aA')
        self.assertEqual(result.a.ab.aba, 'abA')
        self.assertEqual(result.b, 'B')

    def test_simple_list(self):
        result = getParams(['a', 'b'])
        self.assertEqual(result, ['a', 'b'])

    def test_nested_list(self):
        result = getParams([{'a': 'A0'}, {'a': 'A1'}, 'B'])
        a_type = namedtuple(GENERIC_DICT, ('a'))
        self.assertEqual(result, [a_type(a='A0'), a_type(a='A1'), 'B'])
        self.assertEqual(result[0].a, 'A0')
        self.assertEqual(result[1].a, 'A1')
        self.assertEqual(result[2], 'B')

    def test_complex(self):
        result = getParams({'a': 'A', 'b': [{'ba': 'bA0', 'bb': 'bB0'}, {'ba': 'bA1', 'bb': 'bB1'}]})
        b_type = namedtuple(GENERIC_DICT, ('ba', 'bb'))
        self.assertEqual(result, namedtuple(GENERIC_DICT, ('a', 'b'))(a='A', b=[b_type(ba='bA0', bb='bB0'), b_type(ba='bA1', bb='bB1')]))
        self.assertEqual(result.a, 'A')
        self.assertEqual(result.b[0].ba, 'bA0')
        self.assertEqual(result.b[0].bb, 'bB0')
        self.assertEqual(result.b[1].ba, 'bA1')
        self.assertEqual(result.b[1].bb, 'bB1')


if __name__ == '__main__':
    rosunit.unitrun('turtlesim_operator', 'test_params', TestParams)
