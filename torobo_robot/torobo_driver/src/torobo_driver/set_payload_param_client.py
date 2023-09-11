#!/usr/bin/env python
## -*- coding: utf-8 -*-
import rospy
from torobo_msgs.srv import SetPayloadParam
import sys
from os import path 

class SetPayloadParamClient(object):
    def __init__(self, ns, controller_name):
        if (ns[-1] != "/"):
            ns += "/"
        self._ns = ns
        self._controller_name = controller_name
        service_name = self._ns + controller_name + '/set_payload_param'
        rospy.wait_for_service(service_name)
        self.service = rospy.ServiceProxy(service_name, SetPayloadParam)

    def call_service(self, mass, com, inertiaElem):
        try:
            response = self.service(self._controller_name, mass, com, inertiaElem)
            ret = "failed"
            if response.success:
                ret = "succeeded"
            rospy.loginfo('set_payload_param [%s, %f, %s, %s] is %s' % (self._controller_name, mass, map(str, com), map(str, inertiaElem), ret))
        except rospy.ServiceException, e:
            rospy.loginfo("Service call failed: %s" % e)
            return
        return None
    
def main():
    if (len(sys.argv) > 3):
        ns = sys.argv[1]
        controller_name = sys.argv[2]
        mass = float(sys.argv[3])
        com = []
        inertiaElem = []
        if (len(sys.argv) > 6):
            com = [float(sys.argv[4]), float(sys.argv[5]), float(sys.argv[6])]
        if (len(sys.argv) > 12):
            inertiaElem = [float(sys.argv[7]), float(sys.argv[8]), float(sys.argv[9]), float(sys.argv[10]), float(sys.argv[11]), float(sys.argv[12])]
        nodeName = path.splitext(path.basename(__file__))[0]
        rospy.init_node(nodeName)
        client = SetPayloadParamClient(ns, controller_name)
        client.call_service(mass, com, inertiaElem)
    else:
        print "invalid args"

if __name__ == '__main__':
    main()
