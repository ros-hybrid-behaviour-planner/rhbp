"""
Created on 24.05.2017

@author: hrabia
"""

from knowledge_base.knowledge_base_manager import KnowledgeBase
from behaviour_components.behaviours import BehaviourBase

from knowledge_base.knowledge_base_client import KnowledgeBaseClient

import utils.rhbp_logging
rhbplog = utils.rhbp_logging.LogManager(logger_name=utils.rhbp_logging.LOGGER_DEFAULT_NAME + '.kb')


class KnowledgeUpdateBehaviour(BehaviourBase):
    '''
    Behaviour that updates knowledge after it is activated
    '''

    def __init__(self, name, pattern, new_tuple, knowledge_base_name=KnowledgeBase.DEFAULT_NAME, **kwargs):
        """
        :param name: behaviour name
        :param pattern: pattern match that will be updated
        :param new_tuple: new tuple value that will replaced the matched tuples
        :param knowledge_base_name: name of the knowledge base that is used
        :param kwargs: further general BehaviourBase arguments
        """

        super(KnowledgeUpdateBehaviour, self) \
            .__init__(name=name, **kwargs)

        self.pattern = pattern
        self.new_tuple = new_tuple

        self._kb_client = KnowledgeBaseClient(knowledge_base_name=knowledge_base_name)

    def start(self):
        rhbplog.logdebug("Updating knowledge '%s' to '%s'", self.pattern, self.new_tuple)

        self._kb_client.update(self.pattern, self.new_tuple)


