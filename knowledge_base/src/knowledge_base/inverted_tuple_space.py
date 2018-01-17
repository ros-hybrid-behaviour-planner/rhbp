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

    def all(self):
        """
        Return all tuples
        :return:
        """
        res = []
        for k, v in self.tspace.iteritems():
            res.append((k, v))
        return res

    def get(self, pattern, remove=False):
        """
        This is the way to access an item in the tuple space. If the
        item can't be found, raise KeyError. The item can optionally
        be directly removed.
        """
        res = self.many(pattern, 1)
        if not res:
            raise KeyError(pattern)

        resp, res = res[0]
        if remove:
            for i, c in enumerate(res):
                self.colidx[i][c].remove(resp)
            self.lenidx[len(res)].remove(resp)
            self.tspace.pop(resp)
        return res
