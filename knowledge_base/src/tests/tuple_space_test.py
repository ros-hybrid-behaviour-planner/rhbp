from knowledge_base.tuple_space import TupleSpace
import unittest


class TupleSpaceTestSuite(unittest.TestCase):
    def test_add(self):
        tspace = TupleSpace()
        tuple = ('PositionFact', '0', '0')
        tspace.add(tuple)
        self.assertEqual(tuple, tspace.get(tuple), 'Wrong tuple was found')

    def test_find_by_pattern(self):
        tspace = TupleSpace()
        tuple = ('PositionFact', '0', '0')
        tspace.add(tuple)
        readed = tspace.get(('PositionFact', str, str))
        self.assertEqual(tuple, readed, 'Wrong tuple was found')

    def test_dont_find_non_existing(self):
        tspace = TupleSpace()
        try:
            tspace.get(('PositionFact', '0', '0'))
            self.fail('Found tupple')
        except KeyError:
            # TODO: Find better way
            self.assertTrue(True)

    def test_delete_tupple(self):
        tspace = TupleSpace()
        tuple = ('PositionFact', '0', '0')
        tspace.add(tuple)
        readed = tspace.get(tuple, remove=True)
        self.assertEqual(tuple, readed, 'Wrong tuple was found')
        try:
            tspace.get(tuple)
            self.fail('Found tupple')
        except KeyError:
            # TODO: Find better way
            self.assertTrue(True)


if __name__ == '__main__':
    unittest.main()
