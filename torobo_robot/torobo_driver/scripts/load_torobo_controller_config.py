#!/usr/bin/env python

import rospy
import sys
import yaml
import argparse
import os


def main(args):
    if os.path.exists(args.dump_file):
        os.remove(args.dump_file)

    args.robot_config = os.path.join(args.robot_config)
    if not os.path.exists(args.robot_config):
        return
    with open(args.robot_config , 'r') as f:
        robot_config = yaml.load(f)

    args.yaml = os.path.join(args.config_dir, robot_config['model'], args.yaml)
    if not os.path.exists(args.yaml):
        return
    with open(args.yaml, 'r') as f:
        controllers = yaml.load(f)

    data = []
    for key, value in controllers.items():
        data.append(key)
    output = {}
    output['torobo_controller_config'] = data
    
    with open(args.dump_file, "w") as f:
        f.write(yaml.safe_dump(output, default_flow_style=False))
        f.write(yaml.safe_dump(controllers, default_flow_style=False))


def parse_arguments(args):
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot_config", type=str, default="")
    parser.add_argument("--yaml", type=str, default="")
    parser.add_argument("--config_dir", type=str, default="")
    parser.add_argument("--dump_file", type=str, default="/tmp/tmp.yaml")
    return parser.parse_args(args)


if __name__ == '__main__':
    args = parse_arguments(rospy.myargv()[1:])
    main(args)
