"""
Created on 07.12.2016

@author: rieger
"""
from lindypy.TupleSpace import TSpace


class InvertedTupleSpace(TSpace):
    """
    Provides a tuple space, where patterns can stored and searched with a fact
    """

    def find_for_fact(self, fact):
        """
        Code was inspired by TSpace::many
        :param fact: tuple
        :return: all matching patterns
        """
        if len(fact) not in self.lenidx:
            return []
        possible = self.lenidx[len(fact)]

        for idx, val in enumerate(fact):

            possible_for_current_column = []
            try:
                possible_for_current_column.extend(self.colidx[idx][type(val)])
            except KeyError:
                pass

            try:
                possible_for_current_column.extend(self.colidx[idx][val])
            except KeyError:
                pass

            possible = possible.intersection(possible_for_current_column)
            if not possible:
                return []

        return map(lambda id: self.tspace[id], possible)
