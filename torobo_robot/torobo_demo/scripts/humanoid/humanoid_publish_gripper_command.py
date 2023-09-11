#! /usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
import actionlib
import numpy as np
from control_msgs.msg import GripperCommand


def main():

    TOPIC_NAME = '/torobo/left_gripper_controller/command'

    # Initializes a rospy node.
    rospy.init_node('torobo_publish_gripper_command_node')

    # Create a publisher.
    publisher = rospy.Publisher(TOPIC_NAME, GripperCommand, queue_size=1)

    # Wait until the publisher gets ready.
    while publisher.get_num_connections() == 0:
        rospy.sleep(1)

    # open gripper.
    gripper_command(publisher, 0.08, -40.0) # prismatic joint

    rospy.sleep(5)

    # close gripper.
    gripper_command(publisher, 0.02, 40.0) # prismatic joint

    rospy.sleep(5)

    # open gripper.
    gripper_command(publisher, 0.06, -40.0) # prismatic joint

    rospy.sleep(5)

    # close gripper.
    gripper_command(publisher, 0.0, 40.0) # prismatic joint

    rospy.sleep(5)

    rospy.loginfo("finished.")


def gripper_command(publisher, position, max_effort):
    """
    Function for publishing message to move the gripper
        Note that position's unit is "meter" and max_effort's unit is "N".

    Parameters
    ----------
    publisher : rospy.Publisher
        publisher
    position : float
        position of finger.
    max_effort : float
        max effort for grasp

    Returns
    -------
    None

    Throws
    ------
    None
    """

    # Creates a message.
    command = GripperCommand()
    command.position = position
    command.max_effort = max_effort

    # Publish the message.
    publisher.publish(command)


if __name__ == '__main__':
    main()

