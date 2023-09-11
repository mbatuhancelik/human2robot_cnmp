#!/usr/bin/env python

import rospy
import yaml
import argparse
import os

def main(args):
    f = open(os.path.join(args.robot_config) , 'r')
    robot_config = yaml.load(f)
    f.close()

    f = open(os.path.join(args.model_dir, robot_config['model'], args.srdf))
    srdf_text = f.read()
    f.close()

    print srdf_text


def parse_arguments(args):
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot_config", type=str, default="")
    parser.add_argument("--srdf", type=str, default="")
    parser.add_argument("--model_dir", type=str, default="")
    return parser.parse_args(args)


if __name__ == '__main__':
    args = parse_arguments(rospy.myargv()[1:])
    main(args)
