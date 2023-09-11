#!/usr/bin/env python
## -*- coding: utf-8 -*-
import rospy
from torobo_msgs.srv import SetRobotControllerParameter
import sys
from os import path 

class SetRobotControllerParameterClient(object):
    def __init__(self, ns):
        if (ns[-1] != "/"):
            ns += "/"
        self._ns = ns
        self.service = rospy.ServiceProxy(self._ns + 'set_robot_controller_parameter', SetRobotControllerParameter)

    def call_service(self, parameter_name, parameter_values, joint_names):
        rospy.loginfo('set_robot_controller_parameter service client is called')
        if (type(joint_names) == str):
            joint_names = list([joint_names])
        try:
            response = self.service(parameter_name, joint_names, parameter_values)
            ret = "failed"
            if response.success:
                ret = "succeeded"
            rospy.loginfo('set [%s, %s, %s] is %s' % (parameter_name, map(str, parameter_values), joint_names, ret))
        except rospy.ServiceException, e:
            rospy.loginfo("Service call failed: %s" % e)

if __name__ == '__main__':
    if (len(sys.argv) > 4):
        ns = sys.argv[1]
        parameter_name = sys.argv[2]
        parameter_value = float(sys.argv[3])
        joint_names = sys.argv[4:]
        parameter_values = [parameter_value for _ in range(len(joint_names))]

        nodeName = path.splitext(path.basename(__file__))[0]
        rospy.init_node(nodeName)
        client = SetRobotControllerParameterClient(ns)
        client.call_service(parameter_name, parameter_values, joint_names)
    else:
        print "invalid args"