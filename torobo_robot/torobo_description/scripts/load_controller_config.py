#!/usr/bin/env python

import rospy
import sys
import yaml
import argparse
import os


def main(args):
    if os.path.exists(args.dump_file):
        os.remove(args.dump_file)

    f = open(os.path.join(args.robot_config) , 'r')
    robot_config = yaml.load(f)
    f.close()

    f = open(os.path.join(args.model_dir, robot_config['model'], args.yaml), 'r')
    controllers = yaml.load(f)
    f.close()

    data = []
    for key, value in controllers.items():
        data.append(key)
    output = {}
    output['controller_list'] = data
    
    f = open(args.dump_file, "w")
    f.write(yaml.safe_dump(output, default_flow_style=False))
    f.write(yaml.safe_dump(controllers, default_flow_style=False))
    f.close()


def parse_arguments(args):
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot_config", type=str, default="")
    parser.add_argument("--yaml", type=str, default="")
    parser.add_argument("--model_dir", type=str, default="")
    parser.add_argument("--dump_file", type=str, default="/tmp/tmp.yaml")
    return parser.parse_args(args)


if __name__ == '__main__':
    args = parse_arguments(rospy.myargv()[1:])
    main(args)
