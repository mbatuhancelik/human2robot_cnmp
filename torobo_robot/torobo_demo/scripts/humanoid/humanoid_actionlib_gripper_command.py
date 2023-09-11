#! /usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
import actionlib
import numpy as np
from control_msgs.msg import GripperCommandAction, GripperCommandGoal


def main():

    ACTION_SERVICE_NAME = '/torobo/left_gripper_controller/gripper_cmd'

    try:
        # Initializes a rospy node.
        rospy.init_node('torobo_gripper_command_node')

        # Creates the SimpleActionClient.
        action_client = actionlib.SimpleActionClient(ACTION_SERVICE_NAME, GripperCommandAction)

        # Waits until the action server has started up.
        action_client.wait_for_server()

        # open gripper.
        gripper_command(action_client, 0.08, -40.0) # prismatic joint

        # close gripper.
        gripper_command(action_client, 0.02, 40.0) # prismatic joint

        # open gripper.
        gripper_command(action_client, 0.06, -40.0) # prismatic joint

        # close gripper.
        gripper_command(action_client, 0.0, 40.0) # prismatic joint

    except rospy.ROSInterruptException:
        rospy.logerr('ROSInterruptException occurred')

    finally:
        pass

    rospy.loginfo("finished.")


def gripper_command(action_client, position, max_effort):
    """
    Function for action to move the gripper
        Note that position's unit is "meter" and max_effort's unit is "N".

    Parameters
    ----------
    action_client : actionlib.SimpleActionClient
        SimpleActionClient
    position : float
        position of finger.
    max_effort : float
        max effort for grasp

    Returns
    -------
    None

    Throws
    ------
    e : rospy.ROSInterruptException
        action fail
    """

    # Creates a goal.
    goal = GripperCommandGoal()
    goal.command.position = position
    goal.command.max_effort = max_effort

    # Sends the goal.
    action_client.send_goal(goal)

    # Waits for the server.
    finished_before_timeout = action_client.wait_for_result(timeout=rospy.Duration(10.0))
    
    # Log state and result.
    state = action_client.get_state()
    result = action_client.get_result()
    rospy.loginfo('[state ]: ' + str(state))
    rospy.loginfo('[result]: ' + str(result))

    # Check timeout
    if not finished_before_timeout:
        e = rospy.ROSInterruptException('action timeout')
        rospy.logerr(e)
        raise e
    # Check state
    if state != actionlib.GoalStatus.SUCCEEDED:
        e = rospy.ROSInterruptException('action fail')
        rospy.logerr(e)
        raise e


if __name__ == '__main__':
    main()

