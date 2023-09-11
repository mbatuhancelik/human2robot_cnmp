#!/usr/bin/env python

import sys
import rospy
import rosbag
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


PLAY_TOPICS = [
    '/torobo/arm_controller/command',
    '/torobo/left_arm_controller/command',
    '/torobo/right_arm_controller/command',
    '/torobo/torso_controller/command'
    '/torobo/head_controller/command'
]


def main(bagfile):

    rospy.init_node('playbag_node', anonymous=True)

    bag = rosbag.Bag(bagfile, 'r')

    publisher = {}
    for topic in PLAY_TOPICS:
        publisher[topic] = rospy.Publisher(topic, JointTrajectory, queue_size=1)

    rospy.sleep(1)

    try:
        bag_start_time = None
        sim_start_time = None

        for topic, msg, t in bag.read_messages(topics=PLAY_TOPICS):
            if bag_start_time is None:
                bag_start_time = msg.header.stamp
                sim_start_time = rospy.Time.now()
            sim_pub_time = sim_start_time + (msg.header.stamp - bag_start_time)

            msg.header.stamp = rospy.Time(0)  # update timestamp into zero

            while rospy.Time.now() < sim_pub_time:
                rospy.sleep(0.001)

            publisher[topic].publish(msg)
    finally:
        bag.close()


if __name__ == '__main__':
    main(sys.argv[1])

