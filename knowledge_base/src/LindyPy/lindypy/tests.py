import os
import sys
import time
import unittest
import multiprocessing

from lindypy import *

def worker1(client):
    """
    A worker that just outputs one tuple
    """
    client.out((1,2,3,4))

def worker2(client):
    """
    A worker that registers a non-consuming interest
    and answers with a new tuple.
    """
    client.rd((1,2,3,4))
    client.out((5,6,7,8))

def worker3(client):
    """
    A worker that registers a consuming interest
    and answers with a new tuple.
    """
    client.inp((1,2,3,4))
    client.out((7,8,9,10))

def worker4(ts):
    """
    A worker that aborts a consuming interest and then
    just goes on a long computation.
    """
    try:
        with timeout(1):
            ts.inp((1,2,3,4))
    except TimeoutError:
        pass
    time.sleep(20)

def worker5(ts):
    """
    A worker that just raises an exception.
    """
    raise ValueError(55)

class TestCase000Space(unittest.TestCase):

    def testSimple(self):
        from lindypy.TupleSpace import TSpace
        ts = TSpace()
        ts.add((1,2,3,4))
        self.assertEqual(ts.values(), [(1,2,3,4)])
        self.assertEqual(ts.get((1,2,3,4)), (1,2,3,4))
        self.assertRaises(KeyError, ts.get, ((2,3,3,4),))

    def testSimpleRemove(self):
        from lindypy.TupleSpace import TSpace
        ts = TSpace()
        ts.add((1,2,3,4))
        self.assertEqual(ts.values(), [(1,2,3,4)])
        self.assertEqual(ts.get((1,2,3,4), remove=True), (1,2,3,4))
        self.assertRaises(KeyError, ts.get, ((2,3,3,4),))

    def testPatterns(self):
        from lindypy.TupleSpace import TSpace
        ts = TSpace()
        ts.add((1,2,3,4))
        self.assertEqual(ts.values(), [(1,2,3,4)])
        self.assertEqual(ts.get((object,object,object,object)), (1,2,3,4))
        self.assertRaises(KeyError, ts.get, ((object,object,object,object,object),))

    def testComplexPatterns(self):
        from lindypy.TupleSpace import TSpace
        ts = TSpace()
        ts.add((1,2,3,4))
        ts.add((5,2,3,7))
        ts.add((1,2,3,4,5,6))
        self.assertEqual(ts.get((object,2,3,object)), (1,2,3,4))
        self.assertEqual(ts.get((5,object,3,object)), (5,2,3,7))
        self.assertEqual(ts.get((1,2,object,object,5,object)), (1,2,3,4,5,6))
        self.assertRaises(KeyError, ts.get, ((7,6,object,object,object,object),))

    def testFuncPatterns(self):
        from lindypy.TupleSpace import TSpace

        def match_even(x):
            return x%2 == 0

        def match_odd(x):
            return x%2 == 1

        ts = TSpace()
        ts.add((2,1,4))
        ts.add((1,3,6))
        self.assertRaises(KeyError, ts.get, ((match_even,match_even,match_even),))
        self.assertEqual(ts.get((match_even,match_odd,match_even)), (2,1,4))
        self.assertEqual(ts.get((match_odd,match_odd,match_even)), (1,3,6))
        
class TestCase020Timeouts(unittest.TestCase):

    def testTimeout(self):
        def thunk():
            with timeout(1):
                time.sleep(2)
        self.assertRaises(TimeoutError, thunk)

class TestCase050Interests(unittest.TestCase):

    def testInterests(self):
        from lindypy.TupleSpace import Interests
        i = Interests()
        i.add(1234, (1,2,3,4))
        i.add(4711, (5,6,7,8))
        i.add(815, (5,object,object,8))
        l1 = i.interested((1,2,3,4), 999)
        l2 = i.interested((5,6,7,8), 999)
        l2.sort()
        self.assertEqual(l1, [1234])
        self.assertEqual(l2, [815, 4711])

class TestCase100Sync(unittest.TestCase):

    def testOut(self):
        from lindypy.TupleSpace import Manager, Client
        (pipe, other) = multiprocessing.Pipe()
        pipe.send(('out', (1,2,3,4)))
        pipe.send(('shutdown', ()))
        m = Manager(Client, os.getpid(), other)
        self.assertEqual(m.tspace.values(), [(1,2,3,4)])
        
    def testRdGeneric(self):
        from lindypy.TupleSpace import Manager, Client
        (pipe, other) = multiprocessing.Pipe()
        pipe.send(('out', (1,2,3,4)))
        pipe.send(('out', (5,7,8,9)))
        pipe.send(('rd', (object, object, object, object)))
        pipe.send(('rd', (1, object, object, 4)))
        pipe.send(('rd', (object, 2, object, 4)))
        pipe.send(('shutdown', ()))
        m = Manager(Client, os.getpid(), other)
        self.assertEqual(pipe.recv(), (1,2,3,4))
        self.assertEqual(pipe.recv(), (1,2,3,4))
        self.assertEqual(pipe.recv(), (1,2,3,4))
        
    def testInpGeneric(self):
        from lindypy.TupleSpace import Manager, Client
        (pipe, other) = multiprocessing.Pipe()
        pipe.send(('out', (1,2,3,4)))
        pipe.send(('inp', (object, object, object, object)))
        pipe.send(('shutdown', ()))
        m = Manager(Client, os.getpid(), other)
        self.assertEqual(pipe.recv(), (1,2,3,4))
        
    def testInpGenericConsumingInterest(self):
        from lindypy.TupleSpace import Manager, Client
        (pipe, other) = multiprocessing.Pipe()
        pipe.send(('inp', (object, object, object, object)))
        pipe.send(('out', (1,2,3,4)))
        pipe.send(('shutdown', ()))
        m = Manager(Client, os.getpid(), other)
        self.assertEqual(pipe.recv(), (1,2,3,4))
        
class TestCase200Async(unittest.TestCase):

    def testAsync1(self):
        with tuplespace() as ts:
            ts.out((1,2,3,4))
            res = ts.inp((object, object, object, object))
            self.assertEqual(res, (1,2,3,4))

    def testAsync2OutInWorker(self):
        with tuplespace() as ts:
            ts.eval(worker1)
            res = ts.rd((object, object, object, object))
            self.assertEqual(res, (1,2,3,4))
            res = ts.inp((object, object, object, object))
            self.assertEqual(res, (1,2,3,4))

    def testAsync3RdInWorker(self):
        with tuplespace() as ts:
            ts.eval(worker2)
            ts.out((1,2,3,4))
            res = ts.inp((5, object, object, object))
            self.assertEqual(res, (5,6,7,8))
            res = ts.inp((object, object, object, object))
            self.assertEqual(res, (1,2,3,4))

    def testAsync4InpInWorker(self):
        with tuplespace() as ts:
            ts.eval(worker3)
            time.sleep(1.0)
            l = ts.interests()
            self.assertEqual(len(l), 1)
            ts.out((1,2,3,4))
            res = ts.inp((object, object, object, object))
            self.assertEqual(res, (7,8,9,10))

    def testAsync5TypePattern(self):
        with tuplespace() as ts:
            ts.out((1,2,3,4))
            t = ts.inp((int,int,object,object))
            self.assertEqual(t, (1,2,3,4))

    def testRdAndInp(self):
        with tuplespace() as ts:
            ts.eval(worker2)
            ts.eval(worker3)
            time.sleep(1.0)
            l = ts.interests()
            self.assertEqual(len(l), 2)
            ts.out((1,2,3,4))
            l = ts.interests()
            self.assertEqual(len(l), 0)
            res1 = ts.inp((5, object, object, object))
            res2 = ts.inp((7, object, object, object))
            self.assertEqual(res1, (5,6,7,8))
            self.assertEqual(res2, (7,8,9,10))

    def testShutdownWithWorkers(self):
        ts = tuplespace()
        ts.eval(worker3)
        ts.eval(worker2)
        with timeout(1):
            ts.shutdown()
        self.assertTrue(True)

    def testAbortedInp(self):
        """
        Aborting an interest should not capture tuples
        and deliver them on the next inp/rd.
        """
        with tuplespace() as ts:
            try:
                with timeout(1):
                    ts.inp((1,2,3,4))
            except TimeoutError:
                pass
            ts.out((1,2,3,4))
            time.sleep(1)
            def thunk():
                with timeout(1):
                    t = ts.inp((5,6,7,8))
                    self.assertEqual(t, (5,6,7,8))
            self.assertRaises(TimeoutError, thunk)

    def testAbortedInp2(self):
        """
        Aborting an interest has to make sure that no
        tuple is consumed by it after aborting.
        """
        with tuplespace() as ts:
            try:
                with timeout(1):
                    ts.inp((1,2,3,4))
            except TimeoutError:
                pass
            ts.out((1,2,3,4))
            time.sleep(1)
            with timeout(1):
                t = ts.inp((1,2,3,4))
            self.assertEqual(t, (1,2,3,4))

    def testAbortedInp3(self):
        """
        Aborting an interest has to make sure that no
        tuple is consumed by it after aborting. This
        variant works with an aborted interest in
        a different process.
        """
        with tuplespace() as ts:
            ts.eval(worker4)
            time.sleep(3)
            ts.out((1,2,3,4))
            time.sleep(1)
            with timeout(1):
                t = ts.inp((1,2,3,4))
            self.assertEqual(t, (1,2,3,4))

    def testBreakingWorker(self):
        with tuplespace() as ts:
            ts.eval(worker5)
            time.sleep(1)
            t = ts.inp((Exception, object))
            self.assertTrue(isinstance(t[0], ValueError))

