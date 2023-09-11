#!/usr/bin/env python
## -*- coding: utf-8 -*-
import rospy
from torobo_msgs.srv import SendCommonCommand
import sys
from os import path

class SendCommonCommandClient(object):
    def __init__(self, ns):
        if (ns[-1] != "/"):
            ns += "/"
        self._ns = ns
        self.service = rospy.ServiceProxy(self._ns + '/send_common_command', SendCommonCommand)

    def call_service(self, joint_names, whole_order, joint_order, value1=0.0, value2=0.0, value3=0.0, value4=0.0):
        rospy.loginfo('send common command service client is called')
        if (type(joint_names) == str):
            joint_names = list([joint_names])
        try:
            response = self.service(joint_names, whole_order, joint_order, value1, value2, value3, value4)
            if response.success:
                rospy.loginfo('set [%s, %d, %d, %f, %f, %f, %f] success' % (joint_names, whole_order, joint_order, value1, value2, value3, value4))
            else:
                rospy.loginfo('set [%s, %d, %d, %f, %f, %f, %f] failed' % (joint_names, whole_order, joint_order, value1, value2, value3, value4))
        except rospy.ServiceException, e:
            rospy.loginfo("Service call failed: %s" % e)

if __name__ == '__main__':
    if (len(sys.argv) > 8):
        ns = sys.argv[1]
        whole_order = int(sys.argv[2])
        joint_order = int(sys.argv[3])
        value1 = float(sys.argv[4])
        value2 = float(sys.argv[5])
        value3 = float(sys.argv[6])
        value4 = float(sys.argv[7])
        joint_names = sys.argv[8:]
        nodeName = path.splitext(path.basename(__file__))[0]
        rospy.init_node(nodeName)
        client = SendCommonCommandClient(ns)
        client.call_service(joint_names, whole_order, joint_order, value1, value2, value3, value4)
    else:
        print "invalid args"