# -*- coding: utf-8 -*-
"""
Created on Mon Aug 22 16:06:12 2016

@author: Christian Dondrup
"""

import rospy
import rosplan_pnp_bridge.gen_bridge as gb
from rosplan_pnp_bridge.msg import ROSPlanAction
from pnp_plugin_server.pnp_simple_plugin_server import PNPSimplePluginServer


class ROSPlanFeedbackServer(object):
    def __init__(self):
        rospy.loginfo("Starting '%s'." % gb.ACTION_ID)
        self._sps = PNPSimplePluginServer(
            gb.ACTION_ID,
            ROSPlanAction,
            self.execute_cb,
            auto_start=False
        )
        self._sps.start()
        rospy.loginfo("Started '%s'." % gb.ACTION_ID)

    def execute_cb(self, goal):
        print goal