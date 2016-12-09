"""
The definition of simple Tuple Spaces in Python, managed
by a process that owns the data structure and only
communicates via message pipes.
"""

import collections


class MessageException(Exception): pass


class MessageNotUnderstoodError(MessageException): pass


class TupleSpace(object):
    """
    The actual tuple space itself. This class handles optimized
    access to tuples by patterns. Patterns are turned into signatures
    that allow faster lookup. The system has a bag semantic - you can
    put an identical tuple into the system multiple times, it will
    only be in there once and counted.
    """

    def __init__(self):
        self.count = 0
        self.tspace = {}
        self.lenidx = {}
        self.colidx = []

    def many(self, pattern, number):
        """
        Helper-Function returning a list of matching tuples and their
        IDs in the tuplespace for tools.
        """
        possible = None

        if len(pattern) not in self.lenidx:
            return []
        possible = self.lenidx[len(pattern)]

        typechecks = []
        funcchecks = []
        for idx, val in enumerate(pattern):
            if isinstance(val, type):
                typechecks.append((idx, val))
            elif isinstance(val, collections.Callable):
                funcchecks.append((idx, val))
            else:
                if val not in self.colidx[idx]:
                    return []
                else:
                    possible = possible.intersection(self.colidx[idx][val])
                    if not possible:
                        return []

        if not possible:
            return []

        def matches(tup):
            for i, t in typechecks:
                if not isinstance(tup[i], t):
                    return False
            for i, f in funcchecks:
                if not f(tup[i]):
                    return False
            return True

        res = []
        for p in possible:
            t = self.tspace[p]
            if matches(t):
                res.append((p, t))
                if number > 1:
                    number -= 1
                else:
                    return res

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
        return res

    def add(self, tupl):
        """
        Adds a tuple to the tuple space and to the indexes needed
        for faster lookup.
        """
        # add the tuple under a unique id to the tuplespace
        self.count += 1
        self.tspace[self.count] = tupl

        # add the tuple to the length idx
        if len(tupl) not in self.lenidx:
            self.lenidx[len(tupl)] = set()
        self.lenidx[len(tupl)].add(self.count)

        # add the tuple id to the column value index
        while len(self.colidx) < len(tupl):
            self.colidx.append({})
        for idx, val in enumerate(tupl):
            if val not in self.colidx[idx]:
                self.colidx[idx][val] = set()
            self.colidx[idx][val].add(self.count)

    def values(self):
        """
        Returns a list of all tuples in the tuplespace.
        """
        return list(self.tspace.values())
