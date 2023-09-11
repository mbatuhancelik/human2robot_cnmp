#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import sys
from os import path
from torobo_msgs.srv import CheckCollision 

_service = None

def call_service(nameSpace, jointState):
    global _service
    if (nameSpace[-1] != "/"):
        nameSpace += "/"
    try:
        service_name = nameSpace + 'check_collision'
        rospy.wait_for_service(service_name)
        if _service is None:
            _service = rospy.ServiceProxy(service_name, CheckCollision)
        response = _service(jointState)
        return response.isColliding
    except rospy.ServiceException, e:
        rospy.loginfo("Service call failed: %s" % e)
    return False

if __name__ == '__main__':
    if (len(sys.argv) > 1):
        nameSpace = sys.argv[1]
        nodeName = path.splitext(path.basename(__file__))[0]
        rospy.init_node(nodeName)
        ret = call_service(nameSpace, None)
        retstr = "is not colliding."
        if ret:
            retstr = "is colliding!"
        rospy.loginfo("check collision result: %s" % retstr)
    else:
        print "invalid args"