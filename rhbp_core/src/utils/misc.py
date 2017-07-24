"""
This module contains various general helper functions

moduleauthor:: hrabia
"""

import os

from abc import ABCMeta, abstractmethod

class FinalInitCaller(ABCMeta):
    """
    This decorator allows to implement an after __init__ hook
    This allows to guarantee the execution of special initialisation
    code after the full object (full class hierarchy) was constructed
    and is ready to use

    Just implement final_init() in the (base_)class and
    set metaclass like below
    __metaclass__ = FinalInitCaller
    """
    def __call__(cls, *args, **kwargs):
        """Called when you call MyNewClass() """
        obj = type.__call__(cls, *args, **kwargs)
        obj.final_init()
        return obj

    @abstractmethod
    def final_init(self):
        """
        after __init__ hook that has to be implemented by the user of this class.
        It is called after all __init__ methods have been executed
        """
        raise NotImplementedError()


def make_directory_path_available(dir_path):
    if not (dir_path):
        return
    if not (os.path.exists(dir_path)):
        os.makedirs(dir_path)