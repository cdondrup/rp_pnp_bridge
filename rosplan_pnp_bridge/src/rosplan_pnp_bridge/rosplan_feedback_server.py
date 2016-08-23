# -*- coding: utf-8 -*-
"""
Created on Mon Aug 22 16:06:12 2016

@author: Christian Dondrup
"""

import rospy
from rosplan_pnp_bridge.msg import ROSPlanAction
from pnp_plugin_server.pnp_simple_plugin_server import PNPSimplePluginServer
from rosplan_dispatch_msgs.msg import ActionFeedback

END = "end"
START = "start"
FAIL = "fail"
ACTION_ID = "ROSplanAction"


class ROSPlanFeedbackServer(object):
    def __init__(self):
        rospy.loginfo("Starting '%s'." % ACTION_ID)
        self._sps = PNPSimplePluginServer(
            ACTION_ID,
            ROSPlanAction,
            self.execute_cb,
            auto_start=False
        )
        self.pub = rospy.Publisher(
            "/kcl_rosplan/action_feedback",
            ActionFeedback,
            queue_size=1
        )
        self.f = {
            END: self.set_succeeded,
            START: self.set_enabled,
            FAIL: self.set_failed
        }
        self._sps.start()
        rospy.loginfo("Started '%s'." % ACTION_ID)

    def execute_cb(self, goal):
        rospy.loginfo("Setting action '%s' to '%s'" % (goal.id, goal.action))
        self.f[goal.action](goal.id)
        self._sps.set_succeeded()

    def __publish(self, msg):
        self.pub.publish(msg)

    def __new_feedback_msg(self, id, status):
        return ActionFeedback(action_id=id, status=status)

    def set_succeeded(self, id):
        self.__publish(self.__new_feedback_msg(id, "action achieved"))

    def set_failed(self, id):
        self.__publish(self.__new_feedback_msg(id, "action failed"))

    def set_enabled(self, id):
        self.__publish(self.__new_feedback_msg(id, "action enabled"))
