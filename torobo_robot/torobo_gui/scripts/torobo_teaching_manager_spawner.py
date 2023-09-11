#!/usr/bin/env python


import roslaunch
import rospy
import sys
import yaml
import argparse
import os
import torobo_common.rosparam_loader as rosparam_loader

def main(args):
    rospy.init_node('torobo_teaching_manager_spawner_node', anonymous=True)

    controller_dict = rosparam_loader.get_controller_name_joint_names_dict()
    controller_size = 0
    for k, v in controller_dict.items():
        if "gripper" in k:
            continue
        else:
            controller_size += 1

    if controller_size == 1:
        perspective = os.path.join(args.perspective_dir, 'torobo_teaching_manager_1panel.perspective')
    elif controller_size == 2:
        perspective = os.path.join(args.perspective_dir, 'torobo_teaching_manager_2panel.perspective')
    elif controller_size == 3:
        perspective = os.path.join(args.perspective_dir, 'torobo_teaching_manager_3panel.perspective')
    elif controller_size == 4:
        perspective = os.path.join(args.perspective_dir, 'torobo_teaching_manager_4panel.perspective')
    else:
        rospy.logerr("found torobo controller size is %d. size should be 1 ~ 4" % controller_size)

    node = roslaunch.core.Node(
        package=args.pkg,
        node_type=args.type,
        name=args.name,
        namespace=rospy.get_namespace(),
        respawn=False,
        output=args.output,
        args='--perspective-file ' + perspective
    )

    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()

    process = launch.launch(node)
    # print process.is_alive()
    # process.stop()

    rospy.spin()


def parse_arguments(args):
    parser = argparse.ArgumentParser()
    parser.add_argument("--pkg", type=str, default="")
    parser.add_argument("--type", type=str, default="")
    parser.add_argument("--name", type=str, default="")
    parser.add_argument("--output", type=str, default="")
    parser.add_argument("--perspective_dir", type=str, default="")
    return parser.parse_args(args)


if __name__ == '__main__':
    args = parse_arguments(rospy.myargv()[1:])
    main(args)
