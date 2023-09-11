#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
import torobo_collision_detector.check_collision_client

def callback(jointState):
    isColliding = torobo_collision_detector.check_collision_client.call_service(rospy.get_namespace(), jointState)
    if isColliding:
        rospy.loginfo('is colliding')
    else:
        rospy.loginfo('is not colliding')

def detector():
    sub = rospy.Subscriber(rospy.get_namespace() + '/joint_states', JointState, callback, queue_size=1, tcp_nodelay=True)
    
    r = rospy.Rate(100)
    while not rospy.is_shutdown():
        r.sleep()

if __name__ == '__main__':
    rospy.init_node('collision_detector_test')
    rospy.loginfo('collision detector test')

    try:
        detector()
    except rospy.ROSInterruptException:
        pass
