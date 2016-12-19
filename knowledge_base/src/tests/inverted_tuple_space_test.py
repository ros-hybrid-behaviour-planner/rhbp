#! /usr/bin/env python2
import unittest

from knowledge_base.inverted_tuple_space import InvertedTupleSpace


class TupleSpaceTestSuite(unittest.TestCase):

    def test_simple(self):
        tuple_space = InvertedTupleSpace()
        tup = ('pos', '0', '0')
        tuple_space.add(tup)
        result = tuple_space.find_for_fact(tup)
        self.assertEqual(1, len(result))
        self.assertEqual(tup, result[0])

    def test_simple_pattern(self):
        tuple_space = InvertedTupleSpace()
        pattern = ('pos', str, str)
        tuple_space.add(pattern)
        result = tuple_space.find_for_fact(('pos', '0', '0'))
        self.assertEqual(1, len(result))
        self.assertEqual(pattern, result[0])

    def test_multiple_patterns(self):
        tuple_space = InvertedTupleSpace()
        pattern1 = ('pos', str, str)
        tuple_space.add(pattern1)
        pattern2 = ('pos', '0', str)
        tuple_space.add(pattern2)
        result = tuple_space.find_for_fact(('pos', '0', '0'))
        self.assertEqual(2, len(result))
        self.assertTrue(pattern1 in result)
        self.assertTrue(pattern2 in result)


if __name__ == '__main__':
    unittest.main()
