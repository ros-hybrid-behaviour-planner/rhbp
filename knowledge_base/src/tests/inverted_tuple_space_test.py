#! /usr/bin/env python2

import unittest

from knowledge_base.inverted_tuple_space import InvertedTupleSpace


class InvertedTupleSpaceTestSuite(unittest.TestCase):

    @staticmethod
    def add_not_relevant_patterns(inverted_space):
        inverted_space.add(('AnyPattern', "Which", "Should", "Not", "Found", str))
        inverted_space.add(('pos', '1', str))

    def __run_simple_test(self, inverted_space):
        """
        Tests adding and finding a pattern
        """
        tup = ('pos', '0', '0')
        inverted_space.add(tup)
        result = inverted_space.find_for_fact(tup)
        self.assertEqual(1, len(result))
        self.assertEqual(tup, result[0])

    def test_simple_pattern(self):
        """
        Tests adding and finding a pattern, without other patterns
        """
        self.__run_simple_test(InvertedTupleSpace())

    def test_simple_pattern_with_irrelevant_patterns(self):
        """
        Tests adding and finding a pattern, with other patterns
        """
        inverted_tuple_Space = InvertedTupleSpace()
        InvertedTupleSpaceTestSuite.add_not_relevant_patterns(inverted_tuple_Space)
        self.__run_simple_test(inverted_tuple_Space)

    def test_placeholder(self):
        """
        Tests the space for a pattern with placeholders
        """
        tuple_space = InvertedTupleSpace()

        InvertedTupleSpaceTestSuite.add_not_relevant_patterns(tuple_space)

        pattern = ('pos', str, str)
        tuple_space.add(pattern)
        result = tuple_space.find_for_fact(('pos', '0', '0'))
        self.assertEqual(1, len(result))
        self.assertEqual(pattern, result[0])

    def test_multiple_patterns(self):
        """
        Tests if multiple patterns should found
        """
        tuple_space = InvertedTupleSpace()

        InvertedTupleSpaceTestSuite.add_not_relevant_patterns(tuple_space)

        pattern1 = ('pos', str, str)
        tuple_space.add(pattern1)
        pattern2 = ('pos', '0', str)
        tuple_space.add(pattern2)

    def test_all(self):
        tuple_space = InvertedTupleSpace()

        InvertedTupleSpaceTestSuite.add_not_relevant_patterns(tuple_space)

        result = tuple_space.all()
        self.assertEqual(2, len(result))


if __name__ == '__main__':
    unittest.main()
