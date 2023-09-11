#!/usr/bin/env python
## -*- coding: utf-8 -*-
import rospy
from torobo_msgs.srv import SetControlMode
import sys
from os import path 

class SetControlModeClient(object):
    def __init__(self, ns):
        if (ns[-1] != "/"):
            ns += "/"
        self._ns = ns
        self.service = rospy.ServiceProxy(self._ns + 'set_control_mode', SetControlMode)

    def call_service(self, joint_names, control_mode_name):
        rospy.loginfo('set_control_mode service client is called')
        if (type(joint_names) == str):
            joint_names = list([joint_names])
        try:
            response = self.service(control_mode_name, joint_names)
            ret = "failed"
            if response.success:
                ret = "succeeded"
            rospy.loginfo('set_control_mode [%s] is %s' % (joint_names, ret))
        except rospy.ServiceException, e:
            rospy.loginfo("Service call failed: %s" % e)

if __name__ == '__main__':
    if (len(sys.argv) > 3):
        ns = sys.argv[1]
        control_mode_name = sys.argv[2]
        joint_names = sys.argv[3:]
        nodeName = path.splitext(path.basename(__file__))[0]
        rospy.init_node(nodeName)
        client = SetControlModeClient(ns)
        client.call_service(joint_names, control_mode_name)
    else:
        print "invalid args"