"""
This module contains various general helper functions

moduleauthor:: hrabia
"""

import os
import rospy

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


class LogFileWriter(object):
    """
    Custom log file writer 
    The class takes care of the filename length, illegal characters as well as logging flags
    It allows either append mode or direct writing
    """

    filename_max_length = os.pathconf('.', 'PC_NAME_MAX')

    def __init__(self, path, filename, extension, enable=True):
        """
        :param filename: the filename
        :param extension: file extension 
        """
        self._enable = enable
        self._file_handle = None
        self._path = path

        self.filename = self.clean_filename(extension, filename)

        make_directory_path_available(dir_path=path)

        rospy.logdebug("Creating logfile %s", os.path.abspath(self._path + self.filename))

    def clean_filename(self, extension, filename):
        """
        limit filename length to OS requirements and replace illegal characters
        :param extension: file extension
        :param filename: name of file without path
        :return: corrected filename
        """
        filename = filename[:LogFileWriter.filename_max_length - len(extension)] + extension
        filename = filename.replace('/', '-').replace('\\', '-')
        return filename

    def write(self, data):
        '''
        :param data: the data that should be written to the file
        '''
        if not self._enable:
            return

        try:
            with open(self._path + self.filename, 'w') as outfile:
                outfile.write(data)
                outfile.flush()
        except Exception as e:
            rospy.logerr("Logging failed: %s", e)

    def append(self, data, keep_open=True):
        '''
        :param data: the data that should be written to the file
        :param keep_open: set to false if file handle should not be managed by class afterwards
        '''
        if not self._enable:
            return

        try:
            if keep_open:
                self._file_handle = open(self._path + self.filename, 'a+')
                self._file_handle.write(data)
            else:
                self._close_file()
                with open(self._path + self.filename, 'a+') as outfile:
                    outfile.write(data)
        except Exception as e:
            rospy.logerr("Logging failed: %s", e)

    @property
    def enable(self):
        return self._enable

    @enable.setter
    def enable(self, value):
        self._enable = value

    def __del__(self):
        self._close_file()

    def _close_file(self):
        if self._file_handle:
            self._file_handle.close()