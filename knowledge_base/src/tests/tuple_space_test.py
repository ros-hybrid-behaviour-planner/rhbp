#! /usr/bin/env python2
import unittest

from lindypy.TupleSpace import TSpace


class TupleSpaceTestSuite(unittest.TestCase):
    """
    Tests the used tuple space from library
    """

    def test_add(self):
        tuple_space = TSpace()
        test_tuple = ('PositionFact', '0', '0')
        tuple_space.add(test_tuple)
        self.assertEqual(test_tuple, tuple_space.get(test_tuple), 'Wrong tuple was found')

    def test_find_by_pattern(self):
        tuple_space = TSpace()
        test_tuple = ('PositionFact', '0', '0')
        tuple_space.add(test_tuple)
        readed = tuple_space.get(('PositionFact', str, str))
        self.assertEqual(test_tuple, readed, 'Wrong tuple was found')

    def test_dont_find_non_existing(self):
        tuple_space = TSpace()
        try:
            tuple_space.get(('PositionFact', '0', '0'))
            self.fail('Found tupple')
        except KeyError:
            pass

    def test_delete_tupple(self):
        tuple_space = TSpace()
        test_tuple = ('PositionFact', '0', '0')
        tuple_space.add(test_tuple)
        read = tuple_space.get(test_tuple, remove=True)
        self.assertEqual(test_tuple, read, 'Wrong tuple was found')
        try:
            tuple_space.get(test_tuple)
            self.fail('Found tupple')
        except KeyError:
            pass


if __name__ == '__main__':
    unittest.main()
