#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
Created on Fri Jul 22 13:12:27 2016

@author: cd32
"""

import rospy
from actionlib import SimpleActionClient
from actionlib_msgs.msg import GoalStatus
from topological_navigation.msg import GotoNodeAction, GotoNodeGoal
from pnp_plugin_server.pnp_simple_plugin_server import PNPSimplePluginServer
from pnp_plugin_servers.msg import StringAction


class NavServer(object):
    def __init__(self, name):
        rospy.loginfo("Starting '%s'." % name)
        self._ps = PNPSimplePluginServer(
            name=name,
            ActionSpec=StringAction,
            execute_cb=self.execute_cb,
            auto_start=False
        )
        self._ps.register_preempt_callback(self.preempt_cb)
        self.client = SimpleActionClient('topological_navigation', GotoNodeAction)
        self.client.wait_for_server()
        self._ps.start()
        rospy.loginfo("Done")
        
    def execute_cb(self, goal):
        self.client.send_goal_and_wait(GotoNodeGoal(target=goal.value))
        result = self.client.get_state()
        if result == GoalStatus.PREEMPTED:
            return
        if result == GoalStatus.SUCCEEDED:
            self._ps.set_succeeded()
        else:
            self._ps.set_aborted()
        
    def preempt_cb(self):
        self.client.cancel_all_goals()
        self._ps.set_preempted()
        
if __name__ == "__main__":
    rospy.init_node("goto")
    n = NavServer(rospy.get_name())
    rospy.spin()

