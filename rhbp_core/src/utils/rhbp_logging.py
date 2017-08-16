import logging

LOGGER_DEFAULT_NAME = 'rosout.rhbp'

class LogManager(object):
    """
    Class enabling the creation of custom loggers, for instance per package, module or class
    in the fashion of rospy.logging
    
    By default a sublogger of rosout with rhbp postfix is created
    """

    def __init__(self, logger_name=LOGGER_DEFAULT_NAME):

        self.logdebug = logging.getLogger(logger_name).debug

        self.logwarn = logging.getLogger(logger_name).warning

        self.loginfo = logging.getLogger(logger_name).info

        self.logerr = logging.getLogger(logger_name).error

        self.logfatal = logging.getLogger(logger_name).critical