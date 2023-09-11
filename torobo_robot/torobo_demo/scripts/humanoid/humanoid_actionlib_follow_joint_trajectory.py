#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
import numpy as np
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal


def main():

    ACTION_SERVICE_NAME = '/torobo/left_arm_controller/follow_joint_trajectory'
    JOINT_NAMES = ['left_arm/joint_' + str(i) for i in range(1, 7)] # from joint_1 to joint_6

    try:
        # Initializes a rospy node.
        rospy.init_node('torobo_actionlib_follow_joint_trajectory_node')

        # Creates the SimpleActionClient.
        action_client = actionlib.SimpleActionClient(ACTION_SERVICE_NAME, FollowJointTrajectoryAction)

        # Waits until the action server has started up.
        action_client.wait_for_server()

        # Executes an action.
        follow_joint_trajectory(
            action_client = action_client,
            joint_names = JOINT_NAMES,
            positions = np.radians([30.0, 60.0, 0.0, 0.0, 0.0, 0.0]),
            time_from_start = 5.0
        )

        # Executes an action.
        follow_joint_trajectory(
            action_client = action_client,
            joint_names = JOINT_NAMES,
            positions = np.radians([0.0, 60.0, 0.0, 0.0, 0.0, 0.0]),
            time_from_start = 5.0
        )

        # Executes an action.
        follow_joint_trajectory(
            action_client = action_client,
            joint_names = JOINT_NAMES,
            positions = np.radians([-30.0, 60.0, 0.0, 0.0, 0.0, 0.0]),
            time_from_start = 5.0
        )

        # Executes an action.
        follow_joint_trajectory(
            action_client = action_client,
            joint_names = JOINT_NAMES,
            positions = np.radians([0.0, 60.0, 0.0, 0.0, 0.0, 0.0]),
            time_from_start = 5.0
        )

    except rospy.ROSInterruptException:
        rospy.logerr('ROSInterruptException occurred')

    finally:
        pass

    rospy.loginfo("finished.")


def follow_joint_trajectory(action_client, joint_names, positions, time_from_start):
    """
    Function for action to move the arm

    Parameters
    ----------
    action_client : actionlib.SimpleActionClient
        SimpleActionClient
    joint_names : list
        list of joint names
    positions : list
        list of joint's goal positions(radian)
    time_from_start : float
        transition time from start
    
    Returns
    -------
    None

    Throws
    ------
    e : rospy.ROSInterruptException
        action fail
    """

    # Creates a goal.
    goal = FollowJointTrajectoryGoal()
    goal.goal_time_tolerance = rospy.Time(1.0)
    goal.trajectory.header.stamp = rospy.Time.now()
    goal.trajectory.joint_names = joint_names
    point = JointTrajectoryPoint()
    point.positions = positions
    point.velocities = [0.0 for i in range(len(joint_names))]
    point.accelerations = [0.0 for i in range(len(joint_names))]
    point.effort = [0.0 for i in range(len(joint_names))]
    point.time_from_start = rospy.Duration(time_from_start)
    goal.trajectory.points.append(point)

    # Sends the goal.
    action_client.send_goal(goal)

    # Waits for the server.
    finished_before_timeout = action_client.wait_for_result(timeout=rospy.Duration(time_from_start + 10.0))

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

