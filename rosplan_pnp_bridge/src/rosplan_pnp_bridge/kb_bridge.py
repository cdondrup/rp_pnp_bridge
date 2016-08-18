# -*- coding: utf-8 -*-
"""
Created on Thu Aug 18 16:19:06 2016

@author: cd32
"""

import rospy
import utils as ut
from pnp_knowledgebase.pnp_knowledgebase_abstractclass import PNPKnowledgebaseAbstractclass
from rosplan_knowledge_msgs.srv import KnowledgeUpdateService, KnowledgeUpdateServiceRequest
from rosplan_knowledge_msgs.srv import KnowledgeQueryService, KnowledgeQueryServiceRequest
from rosplan_knowledge_msgs.msg import KnowledgeItem
from diagnostic_msgs.msg import KeyValue



class KnowledgeBaseBridge(PNPKnowledgebaseAbstractclass):
    def __init__(self):
        super(KnowledgeBaseBridge, self).__init__()

    def query_knowledgbase(self, predicate):
        """querry the knowledge base.
        :param predicate: The condition as a string taken from the PNP.
        :return (int) -1 (unknown), 0 (false), or 1 (true)
        """
        cond = predicate.split("__")

        while not rospy.is_shutdown():
            try:
                s = rospy.ServiceProxy(
                    "/kcl_rosplan/query_knowledge_base",
                    KnowledgeQueryService
                )
                s.wait_for_service(timeout=1.)
            except rospy.ROSException:
                rospy.logwarn("Could not communicate with '%s' service. Retrying in 1 second." % "/kcl_rosplan/query_knowledge_base")
                rospy.sleep(1.)
            else:
                req = KnowledgeQueryServiceRequest()
                req.knowledge.append(
                    KnowledgeItem(
                        knowledge_type=KnowledgeItem.FACT,
                        attribute_name=cond[0],
                        values=[KeyValue(value=x) for x in cond[1:]]
                    )
                )
                try:
                    r = s(req)
                except rospy.ROSInterruptException:
                    return
                else:
                    return 1 if r.all_true else 0

    def update_knowledgebase(self, predicate, truth_value):
        """update the knowledge base.
        :param predicate: The condition as a string taken from the PNP.
        :param truth_value: (int) -1 (unknown), 0 (false), 1 (true)
        """
        cond = predicate.split("__")

        while not rospy.is_shutdown():
            try:
                s = rospy.ServiceProxy(
                    "/kcl_rosplan/update_knowledge_base",
                    KnowledgeUpdateService
                )
                s.wait_for_service(timeout=1.)
            except rospy.ROSException:
                rospy.logwarn("Could not communicate with '%s' service. Retrying in 1 second." % "/kcl_rosplan/query_knowledge_base")
                rospy.sleep(1.)
            else:
                req = KnowledgeUpdateServiceRequest()
                req.knowledge = KnowledgeItem(
                    knowledge_type=KnowledgeItem.FACT,
                    attribute_name=cond[0],
                    values=[KeyValue(value=x) for x in cond[1:]]
                )
                try:
                    r = s(req)
                except rospy.ROSInterruptException:
                    return
                else:
                    return 1 if r.all_true else 0
