#!/usr/bin/env python
## -*- coding: utf-8 -*-
import rospy
from torobo_msgs.srv import GetServoState
import sys
from os import path

class GetServoStateClient(object):
    def __init__(self, ns):
        if (ns[-1] != "/"):
            ns += "/"
        self._ns = ns
        self.service = rospy.ServiceProxy(self._ns + 'get_servo_state', GetServoState)

    def call_service(self, joint_names):
        # rospy.loginfo('get_servo_state service client is called')
        if (type(joint_names) == str):
            joint_names = list([joint_names])
        try:
            response = self.service(joint_names)
            return response.is_servo_on
        except rospy.ServiceException, e:
            pass
            # rospy.loginfo("Service call failed: %s" % e)
        return None

if __name__ == '__main__':
    if (len(sys.argv) > 2):
        ns = sys.argv[1]
        joint_names = sys.argv[2:]
        nodeName = path.splitext(path.basename(__file__))[0]
        rospy.init_node(nodeName)
        client = GetServoStateClient(ns)
        ret = client.call_service(joint_names)
        rospy.loginfo('get_servo_state service client result: %s' % (map(str, ret)))
    else:
        print "invalid args"