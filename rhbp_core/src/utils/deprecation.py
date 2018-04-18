
import warnings
import functools
import inspect
import traceback

import utils.rhbp_logging
rhbplog = utils.rhbp_logging.LogManager()

"""
This module includes useful decorators to deal with deprectated functions, it is modified version of code found here:
https://wiki.python.org/moin/PythonDecoratorLibrary#Smart_deprecation_warnings_.28with_valid_filenames.2C_line_numbers.2C_etc..29
"""


class DeprecatedClass(type):
    """
    This is a decorator can be used to mark classes
    as deprecated. It will result in a warning being emitted
    when the class is used.

    set metaclass like below
    __metaclass__ = DeprecatedClass
    """
    def __call__(cls, *args, **kwargs):
        """Called when you call MyNewClass() """
        obj = type.__call__(cls, *args, **kwargs)

        msg = "Call to deprecated class {}. {}".format(cls.__name__, traceback.format_stack())

        rhbplog.logwarn(msg)

        warnings.warn_explicit(msg,
            category=DeprecationWarning,
            filename=inspect.getfile(cls.__class__),
            lineno=int(inspect.getsourcelines(cls.__class__)[1])
        )
        return obj


def deprecated(func):
    """
    This is a decorator which can be used to mark functions
    as deprecated. It will result in a warning being emitted
    when the function is used.
    
    Usage examples:

    @deprecated
    def my_func():
        pass
    
    @other_decorators_must_be_upper
    @deprecated
    def my_func():
        pass
    """
    @functools.wraps(func)
    def new_func(*args, **kwargs):
        msg = "Call to deprecated function {}. {}".format(func.__name__, traceback.format_stack())
        rhbplog.logwarn(msg)

        warnings.warn_explicit(
            msg,
            category=DeprecationWarning,
            filename=func.func_code.co_filename,
            lineno=func.func_code.co_firstlineno + 1
        )
        return func(*args, **kwargs)
    return new_func


def ignore_deprecation_warnings(func):
    """
    This is a decorator which can be used to ignore deprecation warnings
    occurring in a function.
    
    Usage examples:
    
    @ignore_deprecation_warnings
    def some_function_raising_deprecation_warning():
        warnings.warn("This is a deprecation warning.", category=DeprecationWarning)
    
    class SomeClass:
        @ignore_deprecation_warnings
        def some_method_raising_deprecation_warning():
            warnings.warn("This is a deprecationg warning.", category=DeprecationWarning)
    
    """
    def new_func(*args, **kwargs):
        with warnings.catch_warnings():
            warnings.filterwarnings("ignore", category=DeprecationWarning)
            return func(*args, **kwargs)
    new_func.__name__ = func.__name__
    new_func.__doc__ = func.__doc__
    new_func.__dict__.update(func.__dict__)
    return new_func