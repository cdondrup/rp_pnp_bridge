# -*- coding: utf-8 -*-
"""
Created on Thu Aug 18 16:19:06 2016

@author: cd32
"""

from pnp_knowledgebase.pnp_knowledgebase_abstractclass import PNPKnowledgebaseAbstractclass
import rosplan_python_utils.knowledge_base_utils as kb



class KnowledgeBaseBridge(PNPKnowledgebaseAbstractclass):
    def __init__(self):
        super(KnowledgeBaseBridge, self).__init__()

    def query_knowledgbase(self, predicate):
        """querry the knowledge base.
        :param predicate: The condition as a string taken from the PNP.
        :return (int) -1 (unknown), 0 (false), or 1 (true)
        """
        return kb.query(predicate)

    def update_knowledgebase(self, predicate, truth_value):
        """update the knowledge base.
        :param predicate: The condition as a string taken from the PNP.
        :param truth_value: (int) -1 (unknown), 0 (false), 1 (true)
        """
        kb.update(predicate, truth_value)
