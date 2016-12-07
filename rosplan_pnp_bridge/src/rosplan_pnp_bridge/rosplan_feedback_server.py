# -*- coding: utf-8 -*-
"""
Created on Mon Aug 22 16:06:12 2016

@author: Christian Dondrup
"""

import rospy
from actionlib import SimpleActionClient
from rosplan_dispatch_msgs.msg import PlanAction, PlanGoal
from pnp_planning_system.pnp_planning_abstractclass import PNPPlanningAbstractclass
from rosplan_pnp_bridge.msg import ROSPlanAction
from pnp_plugin_server.pnp_simple_plugin_server import PNPSimplePluginServer
from rosplan_dispatch_msgs.msg import ActionFeedback
from std_msgs.msg import String

END = "end"
START = "start"
FAIL = "fail"
ACTION_ID = "ROSplanAction"


class ROSPlanFeedbackServer(PNPPlanningAbstractclass):
    def __init__(self):
        rospy.loginfo("Starting '%s'." % ACTION_ID)
        super(ROSPlanFeedbackServer, self).__init__()
        self._sps = PNPSimplePluginServer(
            ACTION_ID,
            ROSPlanAction,
            self.execute_cb,
            auto_start=False
        )
        self.client = SimpleActionClient("/kcl_rosplan/start_planning", PlanAction)
        self.client.wait_for_server()
        self.pub = rospy.Publisher(
            "/kcl_rosplan/action_feedback",
            ActionFeedback,
            queue_size=1
        )
        self.state_pub = rospy.Publisher(
            "~current_state",
            String,
            queue_size=1,
            latch=False
        )
        self.state_pub.publish("")
        self.commands_pub = rospy.Publisher(
            "/kcl_rosplan/planning_commands",
            String,
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

    def goal_state_reached(self):
        print "#### GOAL"
        self.state_pub.publish("goal")

    def fail_state_reached(self):
        print "#### FAIL"
        self.commands_pub.publish("cancel")
        self.client.send_goal(PlanGoal())
        self.state_pub.publish("fail")
