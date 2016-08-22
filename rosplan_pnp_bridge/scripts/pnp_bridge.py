#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
Created on Thu Jul 28 11:06:58 2016

@author: Christian Dondrup
"""

import rospy
from rosplan_pnp_bridge.gen_bridge import GenBridge
from rosplan_pnp_bridge.kb_bridge import KnowledgeBaseBridge
from rosplan_pnp_bridge.rosplan_feedback_server import ROSPlanFeedbackServer
from rosplan_dispatch_msgs.msg import CompletePlan


class ROSPlanBridge(object):
    def __init__(self, name):
        rospy.loginfo("Starting '%s'." % name)
        self.gen = GenBridge() # Used in callback
        self.fb = ROSPlanFeedbackServer() # Handles the communictaion with the dispatcher just need instance
        self.kb = KnowledgeBaseBridge() # Does all the KB magic just need instance
        rospy.sleep(.5)
        rospy.Subscriber("/kcl_rosplan/plan", CompletePlan, self.callback)
        rospy.loginfo("Done.")

    def callback(self, msg):
        self.gen.generate_and_send_pnml(msg)
        rospy.loginfo("Sending pnml")

if __name__ == "__main__":
    rospy.init_node("rosplan_bridge")
    ROSPlanBridge(rospy.get_name())
    rospy.spin()
