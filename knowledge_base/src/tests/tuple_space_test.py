#! /usr/bin/env python2
from lindypy.TupleSpace import TSpace

import unittest
import rostest


PKG = 'knowledge_base'


class TupleSpaceTestSuite(unittest.TestCase):

    def test_add(self):
        tspace = TSpace()
        test_tuple = ('PositionFact', '0', '0')
        tspace.add(test_tuple)
        self.assertEqual(test_tuple, tspace.get(test_tuple), 'Wrong tuple was found')

    def test_find_by_pattern(self):
        tspace = TSpace()
        test_tuple = ('PositionFact', '0', '0')
        tspace.add(test_tuple)
        readed = tspace.get(('PositionFact', str, str))
        self.assertEqual(test_tuple, readed, 'Wrong tuple was found')

    def test_dont_find_non_existing(self):
        tspace = TSpace()
        try:
            tspace.get(('PositionFact', '0', '0'))
            self.fail('Found tupple')
        except KeyError:
            pass

    def test_delete_tupple(self):
        tspace = TSpace()
        test_tuple = ('PositionFact', '0', '0')
        tspace.add(test_tuple)
        read = tspace.get(test_tuple, remove=True)
        self.assertEqual(test_tuple, read, 'Wrong tuple was found')
        try:
            tspace.get(test_tuple)
            self.fail('Found tupple')
        except KeyError:
            pass


if __name__ == '__main__':
    rostest.rosrun(PKG, 'tuple_space_test', TupleSpaceTestSuite)
