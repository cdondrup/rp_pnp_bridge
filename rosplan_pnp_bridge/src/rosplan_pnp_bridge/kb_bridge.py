# -*- coding: utf-8 -*-
"""
Created on Thu Aug 18 16:19:06 2016

@author: cd32
"""

import rospy
from pnp_knowledgebase.pnp_knowledgebase_abstractclass import PNPKnowledgebaseAbstractclass
from rosplan_knowledge_msgs.srv import KnowledgeUpdateService, KnowledgeUpdateServiceRequest
from rosplan_knowledge_msgs.srv import KnowledgeQueryService, KnowledgeQueryServiceRequest
from rosplan_knowledge_msgs.msg import KnowledgeItem
from rosplan_knowledge_msgs.srv  import GetDomainPredicateDetailsService
from diagnostic_msgs.msg import KeyValue



class KnowledgeBaseBridge(PNPKnowledgebaseAbstractclass):
    def __init__(self):
        super(KnowledgeBaseBridge, self).__init__()

    def __call_service(self, srv_name, srv_type, req):
         while not rospy.is_shutdown():
            try:
                s = rospy.ServiceProxy(
                    srv_name,
                    srv_type
                )
                s.wait_for_service(timeout=1.)
            except rospy.ROSException:
                rospy.logwarn("Could not communicate with '%s' service. Retrying in 1 second." % srv_name)
                rospy.sleep(1.)
            else:
                return s(req)

    def _get_predicate_details(self, name):
        srv_name = "/kcl_rosplan/get_domain_predicate_details"
        while not rospy.is_shutdown():
            try:
                return self.__call_service(
                    srv_name,
                    GetDomainPredicateDetailsService,
                    name
                )
            except rospy.ROSInterruptException:
                rospy.logerr("Communication with '%s' interrupted. Retrying." % srv_name)
                rospy.sleep(1.)

    def query_knowledgbase(self, predicate):
        """querry the knowledge base.
        :param predicate: The condition as a string taken from the PNP.
        :return (int) -1 (unknown), 0 (false), or 1 (true)
        """
        cond = predicate.split("__")
        srv_name = "/kcl_rosplan/query_knowledge_base"
        tp = self._get_predicate_details(cond[0]).predicate.typed_parameters
        if len(tp) != len(cond[1:]):
            rospy.logerr("Fact '%s' should have %s parameters but has only %s as parsed from: '%s'" % (cond[0], len(tp), len(cond[1:])))
            return
        req = KnowledgeQueryServiceRequest()
        req.knowledge.append(
            KnowledgeItem(
                knowledge_type=KnowledgeItem.FACT,
                attribute_name=cond[0],
                values=[KeyValue(key=str(k.key), value=str(v)) for k,v in zip(tp, cond[1:])]
            )
        )
        while not rospy.is_shutdown():
            try:
                r = self.__call_service(
                    srv_name,
                    KnowledgeQueryService,
                    req
                )
            except rospy.ROSInterruptException:
                rospy.logerr("Communication with '%s' interrupted. Retrying." % srv_name)
                rospy.sleep(1.)
            else:
                return 1 if r.all_true else 0

    def update_knowledgebase(self, predicate, truth_value):
        """update the knowledge base.
        :param predicate: The condition as a string taken from the PNP.
        :param truth_value: (int) -1 (unknown), 0 (false), 1 (true)
        """
        cond = predicate.split("__")
        srv_name = "/kcl_rosplan/update_knowledge_base"
        rospy.loginfo("Updating %s %s" % (str(predicate), str(truth_value)))
        req = KnowledgeUpdateServiceRequest()
        req.update_type = req.ADD_KNOWLEDGE if truth_value else req.REMOVE_KNOWLEDGE
        tp = self._get_predicate_details(cond[0]).predicate.typed_parameters
        if len(tp) != len(cond[1:]):
            rospy.logerr("Fact '%s' should have %s parameters but has only %s as parsed from: '%s'" % (cond[0], len(tp), len(cond[1:])))
            return
        req.knowledge = KnowledgeItem(
            knowledge_type=KnowledgeItem.FACT,
            attribute_name=cond[0],
            values=[KeyValue(key=str(k.key), value=str(v)) for k,v in zip(tp, cond[1:])]
        )
        while not rospy.is_shutdown():
            try:
                self.__call_service(
                    srv_name,
                    KnowledgeUpdateService,
                    req
                )
            except rospy.ROSInterruptException:
                rospy.logerr("Communication with '%s' interrupted. Retrying." % srv_name)
                rospy.sleep(1.)
            else:
                return
