#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from torobo_driver import torobo_easy_command

if __name__ == '__main__':
    rospy.init_node('torobo_easy_command_node')

    controller = '/torobo/left_arm_controller'

    # send command
    torobo_easy_command.SendEasyCommandText(controller, "r 1 0")
    # wait a while to send command
    rospy.sleep(0.5)

    # send command
    torobo_easy_command.SendEasyCommandText(controller, "s 1 1")
    # wait a while to send command
    rospy.sleep(0.5)

    # send command
    torobo_easy_command.SendEasyCommandText(controller, "tpts 1 30 5")
    # wait a while to send command
    rospy.sleep(0.5)

    # send command
    torobo_easy_command.SendEasyCommandText(controller, "ts 1")
    # wait a while to send command
    rospy.sleep(7.0)

    # send command
    torobo_easy_command.SendEasyCommandText(controller, "s 1 0")
    # wait a while to send command
    rospy.sleep(0.5)
