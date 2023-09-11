#! /usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
import numpy as np
import copy
import tf
from geometry_msgs.msg import Pose
import moveit_commander


def main():

    rospy.init_node("torobo_moveit_commander_node", anonymous=True)

    # Get move_group_commander.
    arm = get_move_group_commander("left_arm")

    # (1) Moving to zero position by "go" method.
    rospy.loginfo("Moving to zero position.")
    zero_position = {name:0.0 for name in arm.get_active_joints()}
    arm.set_start_state_to_current_state()           # set start
    arm.set_joint_value_target(zero_position)        # set target
    arm.go()                                         # plan and execute
    rospy.loginfo(" --> " + str(arm.get_current_pose().pose))

    # (2) create waypoints
    zero_pos = arm.get_current_pose().pose.position
    zero_rpy = arm.get_current_rpy()
    wp1 = create_pose(zero_pos, zero_rpy, 0.5, -0.5,  0.2, 0.0, 0.0, np.radians(-90.0))
    wp2 = create_pose(zero_pos, zero_rpy, 0.3, -0.5, -0.2, 0.0, 0.0, np.radians(-90.0))
    wp3 = create_pose(zero_pos, zero_rpy, 0.5, -0.5, -0.2, 0.0, 0.0, np.radians(-90.0))

    # (3) Moving to wp1 by "plan and execute" method.
    rospy.loginfo("Moving to wp1 by 'plan and execute' method.")
    arm.set_start_state_to_current_state()           # set start
    arm.set_pose_target(wp1)                         # set target
    plan = arm.plan()                                # plan
    arm.execute(plan)                                # execute
    rospy.loginfo(" --> " + str(arm.get_current_pose().pose))

    # (4) Moving to wp2,wp3 "compute_cartesian_path" method.
    rospy.loginfo("Moving to wp2,wp3 by 'cartesian path' method.")
    waypoints = [wp2, wp3]
    (plan, fraction) = arm.compute_cartesian_path(waypoints, 0.01, 0.0)
    arm.execute(plan, wait=True)
    rospy.loginfo(" --> " + str(arm.get_current_pose().pose))


def get_move_group_commander(name):
    """
    Function for getting moveit's move_group commander

    Parameters
    ----------
    name : string
        move_group's name

    Returns
    -------
    commander : MoveGroupCommander
        moveit's move_group commander
    """

    # Initializing move_group_commander.
    robot = moveit_commander.RobotCommander(robot_description=rospy.get_namespace() + "robot_description" , ns=rospy.get_namespace())
    commander = moveit_commander.MoveGroupCommander(name, robot_description=rospy.get_namespace() +"robot_description", ns=rospy.get_namespace())
    # Settings.
    commander.set_max_velocity_scaling_factor(1.0)
    commander.set_max_acceleration_scaling_factor(1.0)
    commander.set_goal_orientation_tolerance(0.1)
    commander.set_goal_position_tolerance(0.1)
    commander.set_goal_joint_tolerance(0.1)
    commander.set_goal_tolerance(0.1)
    commander.set_num_planning_attempts(20)
    return commander

def create_pose(pos, rpy, dx, dy, dz, droll, dpitch, dyaw):
    q = tf.transformations.quaternion_from_euler(rpy[0]+droll, rpy[1]+dpitch, rpy[2]+dyaw)
    pose = Pose()
    pose.position.x = pos.x + dx
    pose.position.y = pos.y + dy
    pose.position.z = pos.z + dz
    pose.orientation.x = q[0]
    pose.orientation.y = q[1]
    pose.orientation.z = q[2]
    pose.orientation.w = q[3]
    return pose

if __name__ == '__main__':
    main()

