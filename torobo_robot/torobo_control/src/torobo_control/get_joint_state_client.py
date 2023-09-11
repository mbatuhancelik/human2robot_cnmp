# -*- coding: utf-8 -*-
import rospy
from torobo_msgs.srv import GetJointState

_service = None

def call_service(nameSpace):
    global _service
    try:
        if _service is None:
            service_name = nameSpace + '/joint_state_server/get_joint_state'
            rospy.wait_for_service(service_name)
            _service = rospy.ServiceProxy(service_name, GetJointState)
        response = _service()
        return response.jointState
    except rospy.ServiceException, e:
        rospy.loginfo("Service call failed: %s" % e)
    return False
