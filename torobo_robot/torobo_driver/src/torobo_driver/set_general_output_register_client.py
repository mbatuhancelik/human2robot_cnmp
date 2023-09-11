#!/usr/bin/env python
## -*- coding: utf-8 -*-
import rospy
from torobo_msgs.srv import SetGeneralOutputRegister
import sys
from os import path 

class SetGeneralOutputRegisterClient(object):
    def __init__(self, ns):
        if (ns[-1] != "/"):
            ns += "/"
        self._ns = ns
        self.service = rospy.ServiceProxy(self._ns + 'set_general_output_register', SetGeneralOutputRegister)

    def call_service(self, general_register_number, parameter_name, joint_names):
        rospy.loginfo('set_general_output_register service client is called')
        if (type(joint_names) == str):
            joint_names = list([joint_names])
        try:
            response = self.service(general_register_number, parameter_name, joint_names)
            ret = "failed"
            if response.success:
                ret = "succeeded"
            rospy.loginfo('set [%d, %s, %s] is %s' % (general_register_number, parameter_name, joint_names, ret))
        except rospy.ServiceException, e:
            rospy.loginfo("Service call failed: %s" % e)

if __name__ == '__main__':
    if (len(sys.argv) > 3):
        ns = sys.argv[1]
        general_register_number = int(sys.argv[2])
        parameter_name = sys.argv[3]
        joint_names = sys.argv[4:]

        nodeName = path.splitext(path.basename(__file__))[0]
        rospy.init_node(nodeName)
        client = SetGeneralOutputRegisterClient(ns)
        client.call_service(general_register_number, parameter_name, joint_names)
    else:
        print "invalid args"