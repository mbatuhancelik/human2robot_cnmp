#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from torobo_msgs.srv import SetControlMode, ServoOn, ServoOff
from torobo_msgs.msg import ToroboJointState


def main():
    rospy.init_node('torobo_msgs_node')

    controller = '/torobo/left_arm_controller'
    joint_names = ['left_arm/joint_5', 'left_arm/joint_6']

    try:

        # Set control mode
        set_control_mode(controller, 'external_force_following', joint_names)

        # Set servo on
        set_servo_on(controller, joint_names)

        # wait a while
        rospy.sleep(5.0)

        # Set servo off
        set_servo_off(controller, joint_names)

        # Set control mode
        set_control_mode(controller, 'position', joint_names)

    except rospy.ROSInterruptException:
        rospy.logerr('ROSInterruptException occurred')

    finally:
        pass

    rospy.loginfo("finished.")


def set_control_mode(controller, control_mode_name, joint_names):
    """
    Set control mode

    Parameters
    ----------
    control_mode_name : string
        Specify control mode from following
        'position' : Position control mode
        'velocity' : Velocity control mode
        'current' : Current control mode
        'external_force_following' : External force following control mode
    joint_names   : list
        list of target joints' names (string)
    """

    # Check argument
    TRAJ = 0 # position control mode
    VELOCITY = 1 # velocity control mode
    CURRENT = 2 # current control mode
    EXTERNAL_FORCE_FOLLOWING = 5 # external force following control mode
    if control_mode_name == 'position':
        estimated_value = TRAJ
    elif control_mode_name == 'velocity':
        estimated_value = VELOCITY
    elif control_mode_name == 'current':
        estimated_value = CURRENT
    elif control_mode_name == 'external_force_following':
        estimated_value = EXTERNAL_FORCE_FOLLOWING
    else:
        e = rospy.ROSInterruptException('control_mode name is invalid')
        rospy.logerr(e)
        raise e

    # Call service
    service_name = controller + '/set_control_mode'
    rospy.wait_for_service(service_name)
    service = rospy.ServiceProxy(service_name, SetControlMode)
    rospy.loginfo('call set_control_mode service')
    response = service(control_mode_name, joint_names)

    # Check whether send is succeeded
    if not response.success:
        e = rospy.ROSInterruptException('send packet fail')
        rospy.logerr(e)
        raise e

    # Check whether state is updated
    success_flag = True
    for _ in range(10):
        rospy.sleep(0.1)
        msg = rospy.wait_for_message(controller + '/torobo_joint_state', ToroboJointState, 0.5)
        if msg is None:
            continue
        for joint_name in joint_names:
            for i, name in enumerate(msg.name):
                if joint_name == name:
                    if msg.ctrlMode[i] != estimated_value:
                        success_flag = False
        if success_flag:
            break
        else:
            continue

    if not success_flag:
        e = rospy.ROSInterruptException('state is not updated')
        rospy.logerr(e)
        raise e

    rospy.loginfo('set_control_mode service is succeeded')


def set_servo_on(controller, joint_names):
    """
    Set servo on

    Parameters
    ----------
    joint_names   : list
        list of target joints' names (string)
    """

    STOP = 0
    READY = 1
    RUN = 2
    ERROR = 3
    estimated_value = RUN

    # Call service
    service_name = controller + '/servo_on'
    rospy.wait_for_service(service_name)
    service = rospy.ServiceProxy(service_name, ServoOn)
    rospy.loginfo('call set_servo_on service')
    response = service(joint_names)

    # Check whether send is succeeded
    if not response.success:
        e = rospy.ROSInterruptException('send packet fail')
        rospy.logerr(e)
        raise e

    # Check whether state is updated
    success_flag = True
    for _ in range(10):
        rospy.sleep(0.1)
        msg = rospy.wait_for_message(controller + '/torobo_joint_state', ToroboJointState, 0.5)
        if msg is None:
            continue
        for joint_name in joint_names:
            for i, name in enumerate(msg.name):
                if joint_name == name:
                    if msg.systemMode[i] != estimated_value:
                        success_flag = False
        if success_flag:
            break
        else:
            continue

    if not success_flag:
        e = rospy.ROSInterruptException('state is not updated')
        rospy.logerr(e)
        raise e

    rospy.loginfo('set_servo_on service is succeeded')


def set_servo_off(controller, joint_names):
    """
    Set servo off

    Parameters
    ----------
    joint_names   : list
        list of target joints' names (string)
    """

    STOP = 0
    READY = 1
    RUN = 2
    ERROR = 3
    estimated_value = STOP

    # Call service
    service_name = controller + '/servo_off'
    rospy.wait_for_service(service_name)
    service = rospy.ServiceProxy(service_name, ServoOff)
    rospy.loginfo('call set_servo_on service')
    response = service(joint_names)

    # Check whether send is succeeded
    if not response.success:
        e = rospy.ROSInterruptException('send packet fail')
        rospy.logerr(e)
        raise e

    # Check whether state is updated
    success_flag = True
    for _ in range(10):
        rospy.sleep(0.1)
        msg = rospy.wait_for_message(controller + '/torobo_joint_state', ToroboJointState, 0.5)
        if msg is None:
            continue
        for joint_name in joint_names:
            for i, name in enumerate(msg.name):
                if joint_name == name:
                    if msg.systemMode[i] != estimated_value:
                        success_flag = False
        if success_flag:
            break
        else:
            continue

    if not success_flag:
        e = rospy.ROSInterruptException('state is not updated')
        rospy.logerr(e)
        raise e

    rospy.loginfo('set_servo_off service is succeeded')


if __name__ == '__main__':
    main()

