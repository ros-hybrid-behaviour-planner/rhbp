'''
Created on 25.03.2017

@author: rieger
'''

from behaviour_components.sensors import Sensor
from knowledge_base.knowledge_base_manager import KnowledgeBase
from knowledge_base.update_handler import KnowledgeBaseFactCache

import utils.rhbp_logging
rhbplog = utils.rhbp_logging.LogManager(logger_name=utils.rhbp_logging.LOGGER_DEFAULT_NAME + '.kb')

class KnowledgeSensor(Sensor):
    """
    Sensor, which provides information about existence of any fact, matching the given pattern
    """

    def __init__(self, pattern, optional=False, knowledge_base_name=KnowledgeBase.DEFAULT_NAME, name=None):
        super(KnowledgeSensor, self).__init__(name=name, optional=optional, initial_value=None)
        self.__value_cache = KnowledgeBaseFactCache(pattern=pattern, knowledge_base_name=knowledge_base_name)

    def sync(self):
        self.update(self.__value_cache.does_fact_exists())
        super(KnowledgeSensor, self).sync()


class KnowledgeFactSensor(Sensor):
    """
    Sensor, which provides information about a searched fact; returns list of
    all matching facts
    """

    def __init__(self, pattern, optional=False, knowledge_base_name=KnowledgeBase.DEFAULT_NAME,
                 name=None, initial_value=None):
        super(KnowledgeFactSensor, self).__init__(name=name, optional=optional,
                                                  initial_value=initial_value)
        self.__value_cache = KnowledgeBaseFactCache(pattern=pattern,
                                                    knowledge_base_name=
                                                    knowledge_base_name)

    def sync(self):
        self.update(self.__value_cache.get_all_matching_facts())
        super(KnowledgeFactSensor, self).sync()
