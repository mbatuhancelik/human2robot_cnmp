#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from torobo_msgs.srv import GetToroboJointState
import sys
from os import path 

_service = None

def call_service(nameSpace, controllerName):
    global _service
    try:
        if _service is None:
            service_name = nameSpace + '/joint_state_server/get_torobo_joint_state'
            rospy.wait_for_service(service_name)
            _service = rospy.ServiceProxy(service_name, GetToroboJointState)
        response = _service(controllerName)
        return response.toroboJointState
    except rospy.ServiceException, e:
        pass
        # rospy.loginfo("Service call failed: %s" % e)
    return None

if __name__ == '__main__':
    if (len(sys.argv) > 2):
        nameSpace = sys.argv[1]
        controllerName = sys.argv[2]
        nodeName = path.splitext(path.basename(__file__))[0]
        rospy.init_node(nodeName)
        print call_service(nameSpace, controllerName)
    else:
        print "invalid args"