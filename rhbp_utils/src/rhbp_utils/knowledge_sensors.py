"""
Created on 25.03.2017

@author: rieger, hrabia
"""

from behaviour_components.sensors import Sensor, AggregationSensor
from knowledge_base.knowledge_base_manager import KnowledgeBase
from knowledge_base.update_handler import KnowledgeBaseFactCache

import utils.rhbp_logging
rhbplog = utils.rhbp_logging.LogManager(logger_name=utils.rhbp_logging.LOGGER_DEFAULT_NAME + '.kb')


class KnowledgeSensor(Sensor):
    """
    Sensor, which provides information about existence of any fact, matching the given pattern
    """

    def __init__(self, pattern, optional=False, knowledge_base_name=KnowledgeBase.DEFAULT_NAME, name=None):
        super(KnowledgeSensor, self).__init__(name=name, optional=optional, initial_value=False)
        self._value_cache = KnowledgeBaseFactCache(pattern=pattern, knowledge_base_name=knowledge_base_name)
        self._value_cache.add_update_listener(self._cache_update_callback)

    def _cache_update_callback(self):
        self.update(self._value_cache.does_fact_exists())


class KnowledgeFactSensor(Sensor):
    """
    Sensor, which provides information about a searched fact; returns list of
    all matching facts
    """

    def __init__(self, pattern, optional=False, knowledge_base_name=KnowledgeBase.DEFAULT_NAME, name=None,
                 initial_value=None):
        super(KnowledgeFactSensor, self).__init__(name=name, optional=optional, initial_value=initial_value)
        self._value_cache = KnowledgeBaseFactCache(pattern=pattern, knowledge_base_name=knowledge_base_name)
        self._value_cache.add_update_listener(self._cache_update_callback)

    def _cache_update_callback(self):
        self.update(self._value_cache.get_all_matching_facts())


class KnowledgeFirstFactSensor(KnowledgeFactSensor):
    """
    Sensor, which provides the last value of the first found fact tuple, e.g. fact matches (a,b,c), it would return c
    """

    def __init__(self, pattern, optional=False, knowledge_base_name=KnowledgeBase.DEFAULT_NAME,
                 name=None, initial_value=None):
        super(KnowledgeFirstFactSensor, self).__init__(name=name, optional=optional, initial_value=initial_value,
                                                       pattern=pattern, knowledge_base_name=knowledge_base_name)

    def _reduce_facts(self, facts):
        """
        Reduce the tuple of facts to a single value
        :param facts: fact tuple
        :return: single value, e.g. bool, float, str
        """

        value = self._initial_value

        if len(facts) > 0:
            fact_tuple = facts.pop()  # only getting the first fact

            try:
                value = fact_tuple[-1]
            except Exception:
                rhbplog.logwarn("Couldn't get last tuple element of: %s. Resetting to initial_value", str(fact_tuple))

        return value

    def update(self, facts):

        value = self._reduce_facts(facts)

        super(KnowledgeFirstFactSensor, self).update(value)


class KnowledgeFactNumberSensor(KnowledgeFirstFactSensor):
    """
    Knowledge Sensor, which converts the last part of the first tuple fact to a number/float()
    """

    def __init__(self, pattern, optional=False, knowledge_base_name=KnowledgeBase.DEFAULT_NAME,
                 name=None, initial_value=None):
        super(KnowledgeFactNumberSensor, self).__init__(pattern=pattern, knowledge_base_name= knowledge_base_name,
                                                        name=name, optional=optional, initial_value=initial_value)

    def _reduce_facts(self, facts):

        new_value = super(KnowledgeFactNumberSensor, self)._reduce_facts(facts)

        try:
            new_value = float(new_value)
        except Exception:
            rhbplog.logwarn("Couldn't cast tuple element to float: %s. Resetting to initial_value", str(new_value))
            new_value = self._initial_value

        return new_value


class KnowledgeFactCountSensor(KnowledgeFirstFactSensor):
    """
    Sensor, which provides the number of matching facts for a given pattern
    """

    def __init__(self, pattern, optional=False, knowledge_base_name=KnowledgeBase.DEFAULT_NAME,
                 name=None, initial_value=None):
        super(KnowledgeFactCountSensor, self).__init__(name=name, optional=optional, initial_value=initial_value,
                                                       pattern=pattern, knowledge_base_name=knowledge_base_name)

    def _reduce_facts(self, facts):
        """
        Reduce the tuple of facts to a single value
        :param facts: fact tuple
        :return: single value, e.g. bool, float, str
        """

        return len(facts)
