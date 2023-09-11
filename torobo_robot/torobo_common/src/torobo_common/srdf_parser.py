#!/usr/bin/env python

import rospy
import sys
import yaml
import argparse
import os

import xml.dom.minidom
from math import pi

def get_group_state_from_rosparam(description_param_name="robot_description_semantic"):
    description = rospy.get_param(description_param_name, None)
    if(description is None):
        return None
    return get_group_state_from_text(description)

def get_group_state_from_text(description_text):
    import xml.dom.minidom
    # Code inspired on the joint_state_publisher package by David Lu!!!
    # https://github.com/ros/robot_model/blob/indigo-devel/
    # joint_state_publisher/joint_state_publisher/joint_state_publisher
    robot = xml.dom.minidom.parseString(description_text).getElementsByTagName('robot')[0]

    # Find all non-fixed joints
    output = {}
    for child in robot.childNodes:
        if child.nodeType is child.TEXT_NODE:
            continue
        if child.localName != 'group_state':
            continue

        group = child.getAttribute('group')
        if group not in output:
            output[group] = {}
        name = child.getAttribute('name')
        output[group][name] = {}
        output[group][name]["joint_names"] = []
        output[group][name]["positions"] = []

        joints = child.getElementsByTagName('joint')
        for joint in joints:
            joint_name = joint.getAttribute('name')
            joint_value = joint.getAttribute('value')
            output[group][name]["joint_names"].append(joint_name)
            output[group][name]["positions"].append(float(joint_value))
    return output

if __name__ == '__main__':
    state = get_group_state_from_rosparam()
    for group, names in state.items():
        print group
        for k, v in names.items():
            print "  - ", k, v
