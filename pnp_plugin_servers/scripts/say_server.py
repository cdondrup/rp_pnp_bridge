#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
Created on Fri Jul 22 13:12:27 2016

@author: cd32
"""

import rospy
from pnp_plugin_server.pnp_simple_plugin_server import PNPSimplePluginServer
from pnp_plugin_servers.msg import StringAction, StringResult
from std_msgs.msg import String


class SayServer(object):
    
    __dict = {
        "hello": "Hello, I am pepper",
        "bye": "Asta la vista, baby"
    }    
    
    def __init__(self, name):
        rospy.loginfo("Starting '%s'." % name)
        self._ps = PNPSimplePluginServer(
            name=name,
            ActionSpec=StringAction,
            execute_cb=self.execute_cb,
            auto_start=False
        )
        self.pub = rospy.Publisher("/speech", String, queue_size=1)
        self._ps.start()
        rospy.loginfo("Done")
        
    def execute_cb(self, goal):
        text = self.__dict[goal.value]
        print "Say:", text
        self.pub.publish(text)
        self._ps.set_succeeded(StringResult(goal.value))
        
if __name__ == "__main__":
    rospy.init_node("say")
    n = SayServer(rospy.get_name())
    rospy.spin()

