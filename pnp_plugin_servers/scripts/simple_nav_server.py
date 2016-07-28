#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
Created on Fri Jul 22 13:12:27 2016

@author: cd32
"""

import rospy
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Quaternion
from pnp_plugin_server.pnp_simple_plugin_server import PNPSimplePluginServer
from pnp_plugin_servers.msg import StringAction, StringResult
import tf
import numpy as np


ODOM = "odom"
BASE_LINK = "base_link"

class NavServer(object):
    def __init__(self, name):
        rospy.loginfo("Starting '%s'." % name)
        self._ps = PNPSimplePluginServer(
            name=name,
            ActionSpec=StringAction,
            execute_cb=self.execute_cb,
            auto_start=False
        )
        self.listener = tf.TransformListener()
        self.pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
        self._ps.start()
        rospy.loginfo("Done")
        
    def get_transfrom(self, f1, f2):
        try:
            (trans,rot) = self.listener.lookupTransform(f1, f2, rospy.Time(0))
            return trans, rot
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            if not rospy.is_shutdown() and not self._ps.is_preempt_requested():
                rospy.sleep(0.1)
                return self.get_transfrom(f1, f2)
                
    def get_euclidean_distance(self, p1, p2):
        return np.sqrt(
            np.square(p1[0] - p2[0]) \
            + np.square(p1[1] - p2[1])
        )

    def execute_cb(self, goal):
        p = PoseStamped()
        p.header.stamp = rospy.Time.now()
        p.header.frame_id = BASE_LINK
        start, r = self.get_transfrom(BASE_LINK, ODOM)
        p.pose.orientation = Quaternion(x=r[0], y=r[1], z=r[2], w=r[3])
        if goal.value == "close":
            p.pose.position.x = 1.0
        elif goal.value == "far":
            p.pose.position.x = -1.0
        elif goal.value == "left":
            p.pose.position.y = 1.0
        elif goal.value == "right":
            p.pose.position.y = -1.0
        self.pub.publish(p)
        
        timeout = p.header.stamp.to_sec() + 3.
        while not rospy.is_shutdown() and not self._ps.is_preempt_requested() \
                and timeout > rospy.Time.now().to_sec():
            t, _ = self.get_transfrom(BASE_LINK, ODOM)
            if self.get_euclidean_distance(start[:2], t[:2]) > .95:
                break
            rospy.sleep(.1)
        
        self._ps.set_succeeded(StringResult(goal.value))
        
if __name__ == "__main__":
    rospy.init_node("goto")
    n = NavServer(rospy.get_name())
    rospy.spin()

