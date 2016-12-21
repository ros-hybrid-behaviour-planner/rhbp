"""
The definition of simple Tuple Spaces in Python, managed
by a process that owns the data structure and only
communicates via message pipes.
"""

import os
import sys
import select
import signal
import traceback
import multiprocessing

class MessageException(Exception): pass
class MessageNotUnderstoodError(MessageException): pass

class TSpace(object):
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
            elif callable(val):
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
            for i,t in typechecks:
                if not isinstance(tup[i], t):
                    return False
            for i,f in funcchecks:
                if not f(tup[i]):
                    return False
            return True

        res = []
        for p in possible:
            t = self.tspace[p]
            if matches(t):
                res.append((p,t))
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
            for i,c in enumerate(res):
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
        return self.tspace.values()

class Interests(object):
    """
    Handles registered interests of processes efficiently.
    """
    def __init__(self):
        self.patterns = {}
        self.lenidx = {}

    def empty(self):
        return len(self.patterns) == 0

    def items(self):
        """
        Returns all registered interests.
        """
        return self.patterns.items()

    def add(self, pid, pattern):
        """
        Add a new interest to the registry.
        """
        self.patterns[pid] = pattern
        if len(pattern) not in self.lenidx:
            self.lenidx[len(pattern)] = set()
        self.lenidx[len(pattern)].add(pid)

    def interested(self, tupl, number):
        """
        Return a list of pids that are interested in a pattern.
        """
        if len(tupl) not in self.lenidx:
            return []
        possible = self.lenidx[len(tupl)]

        def matches(pat):
            for p, v in zip(pat, tupl):
                if isinstance(p, type):
                    if not isinstance(v, p):
                        return False
                elif callable(p):
                    if not p(v):
                        return False
                else:
                    if p != v:
                        return False
            return True

        res = []
        for pid in possible:
            if matches(self.patterns[pid]):
                res.append(pid)
                if number > 1:
                    number -= 1
                elif number >= 0:
                    return res
        return res

    def remove(self, pid):
        """
        Remove a pid from the interests because you delivered a tuple.
        Fails silently if pid is not registered as an interest.
        """
        if pid in self.patterns:
            self.lenidx[len(self.patterns[pid])].remove(pid)
            del self.patterns[pid]

class Manager(object):
    """
    The managing class for a tuple space. It gets the class of clients
    to produce and the pid and pipe of the main process.
    
    The actual messages are tuples of a message tag and the actual
    tuple.

    A tuple space starts empty, waiting for messages. It keeps
    running as long as there is anyone interested in tuples and ends
    when there is nobody around listening anymore.
    """
    def __init__(self, clientcls, mainpid, mainpipe):
        # keep track of the class to create clients
        self.clientcls = clientcls

        # keep track of the main pid, since that is not in our active children
        self.mainpid = mainpid

        # the register of processes waiting on this tuplespace. Key is the
        # process ID, value is the pipe to communicate with that pid.
        self.registered = {}
        self.registered[mainpid] = mainpipe

        # consuming and non-consuming interests for tuples.
        self.consuming_int = Interests()
        self.non_consuming_int = Interests()

        # the tuple space itself. Key is the lenght of the tuples
        # and value is a set of tuples.
        self.tspace = TSpace()

        # start the mainloop
        self.run()

    def thunk(self, fun, client):
        """
        This is meant to build an environment to catch
        errors in workers and send them to some central
        monitoring instance.
        """
        try:
            fun(client)
        except Exception, e:
            typ, val, tb = sys.exc_info()
            s = tuple(tuple(x) for x in traceback.extract_tb(tb))
            tb = None
            client.out((e,s))

    def next(self):
        """
        Return the next messages from the pipes that connect the
        manager to the clients. Timeout at 1 sec to reap dead child
        processes.
        """
        msg = []
        while self.registered and not msg:
            pipes = dict((self.registered[pid].fileno(), pid) for pid in self.registered)
            (ready, _, _) = select.select(pipes.keys(), [], [], 1)
            for p in ready:
                pid = pipes[p]
                msg.append((pid, self.registered[pid].recv()))
            pids = set(proc.pid for proc in multiprocessing.active_children())
            pids.add(self.mainpid)
            for pid in self.registered.keys():
                if pid not in pids:
                    del self.registered[pid]
        return msg

    def handle_shutdown(self, pid, tup):
        """
        Shut down the tuple space, kill all still active workers and
        end the manager thread.
        """
        for proc in multiprocessing.active_children():
            os.kill(proc.pid, signal.SIGTERM)
        self.registered = {}

    def handle_interests(self, pid, tup):
        """
        Return a list of tuples (pid, consuming?, pattern) of interests.
        Only used for debugging and from the main thread.
        """
        res = []
        for (ipid, pat) in self.consuming_int.items():
            res.append((ipid, True, pat))
        for (ipid, pat) in self.non_consuming_int.items():
            res.append((ipid, False, pat))
        self.registered[pid].send(res)

    def handle_eval(self, pid, tup):
        """
        fork a process and keep track in our process dict
        the actual function must be the import specification
        for a callable in absolute notation.
        """
        q = multiprocessing.Pipe()
        client = self.clientcls(manager=self, pipe=q[1])
        proc = multiprocessing.Process(target=self.thunk, args=(tup[0], client))
        proc.start()
        self.registered[proc.pid] = q[0]

    def handle_inp(self, pid, pattern):
        """
        look for a matching tuple in the process and return it,
        otherwise register a consuming interest for a tuple pattern.
        """
        try:
            t = self.tspace.get(pattern, remove=True)
        except KeyError:
            self.consuming_int.add(pid, pattern)
            return
        self.registered[pid].send(t)

    def handle_rd(self, pid, pattern):
        """
        look for a matching tuple in the process and return it. If
        there is none, register a non-consuming interest.
        """
        try:
            t = self.tspace.get(pattern)
        except KeyError:
            self.non_consuming_int.add(pid, pattern)
            return
        self.registered[pid].send(t)

    def handle_unreg(self, pid, tup):
        """
        unregister interest. Tuple has one bool that tells us if it
        was consuming or nonconsuming.
        """
        if tup[0]:
            self.consuming_int.remove(pid)
        else:
            self.non_consuming_int.remove(pid)

    def handle_out(self, pid, tup):
        """
        Check for non-consuming interests and if there are any, fullfill them.
        Check for consuming interests and if there is one, fullfill that
        interest. Otherwise put the tuple into the tuple space.
        """
        if not self.non_consuming_int.empty():
            for ipid in self.non_consuming_int.interested(tup, -1):
                self.registered[ipid].send(tup)
                self.non_consuming_int.remove(ipid)

        if not self.consuming_int.empty():
            ipid = self.consuming_int.interested(tup, 1)
            if ipid:
                ipid = ipid[0]
                self.registered[ipid].send(tup)
                self.consuming_int.remove(ipid)
                return

        self.tspace.add(tup)

    def run(self):
        while self.registered:
            for pid, (cmd, tup) in self.next():
                cmd = 'handle_' + cmd
                getattr(self, cmd)(pid, tup)

class Client(object):
    """
    A Tuple Space is an abstraction that can keep a number
    of tuples of arbitrary structure. Tuple space objects
    are shared between processes
    """
    def __init__(self, manager=None, pipe=None):
        """
        Initialize a client connection and start a tuple space
        manager process.
        """
        if manager is None and pipe is None:
            self.pipe, managerpipe = multiprocessing.Pipe()
            self.manager = multiprocessing.Process(target=Manager,
                args=(self.__class__, os.getpid(), managerpipe))
            self.manager.start()
            self.ismain = True
        elif manager is not None and pipe is not None:
            self.pipe = pipe
            self.manager = manager
            self.ismain = False
        else:
            raise TypeError("either initialize a tuplespace client with all values or no values")
  
    def __enter__(self):
        return self

    def __exit__(self, *args):
        if self.ismain:
            self.shutdown()

    def inp(self, pat, consuming=True):
        """
        Look for a tuple matching the given pattern. If there
        is none, block. If there is one, return it and remove
        it from the tuple space.
        """
        # grab remaining tuples from the pipe and reinsert
        # them, as they are from aborted interests
        while self.pipe.poll():
            self.out(self.pipe.recv())
        self.pipe.send(('inp' if consuming else 'rd', pat))
        try:
            return self.pipe.recv()
        except:
            self.pipe.send(('unreg', (consuming,)))
            raise

    def rd(self, pat):
        """
        Look for a tuple matching the given pattern. If there
        is none, block. If there is one, return it without
        consuming it from the tuple space.
        """
        return self.inp(pat, consuming=False)

    def out(self, tup):
        """
        Put a given tuple into the tuple space atomically.
        """
        self.pipe.send(('out', tup))

    def eval(self, fun):
        """
        Start a new process and pass a tuple space client as the first
        parameter.  That new process can communicate with the tuple space
        in the same way as anybody else.
        """
        self.pipe.send(('eval', (fun,)))

    def interests(self):
        """
        Returns a list of tuples (pid, consuming?, pattern) of interests.
        Can only be invoked by the main thread and is mostly used for testing.
        """
        assert self.ismain, "only main threads are allowed to query manager state"
        self.pipe.send(('interests', ()))
        return self.pipe.recv()
    
    def shutdown(self):
        """
        Shut down the tuple space and kill all running workers.
        """
        assert self.ismain, "only main threads are allowed to shutdown the tuple space"
        self.pipe.send(('shutdown', ()))
        self.manager.join()
    
def tuplespace():
    """
    Create a new tuple space and return the client
    object.
    """
    return Client()

