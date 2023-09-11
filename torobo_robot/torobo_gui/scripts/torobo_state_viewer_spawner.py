#!/usr/bin/env python


import roslaunch
import rospy
import sys
import yaml
import argparse
import os


def main(args):
    rospy.init_node('torobo_state_viewer_spawner_node', anonymous=True)

    torobo_controller_config = rospy.get_param('torobo_controller_config', None)
    if(torobo_controller_config is None):
        rospy.logerr("rosparam [%s] is not found." % (rospy.get_namespace() + "torobo_controller_config"))
        return

    controller_size = len(torobo_controller_config)
    if controller_size == 1:
        perspective = os.path.join(args.perspective_dir, 'torobo_state_viewer_1panel.perspective')
    elif controller_size == 2:
        perspective = os.path.join(args.perspective_dir, 'torobo_state_viewer_2panel.perspective')
    elif controller_size == 3:
        perspective = os.path.join(args.perspective_dir, 'torobo_state_viewer_3panel.perspective')
    else:
        rospy.logerr("found torobo controller size is %d. size should be 1 ~ 3" % controller_size)

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
