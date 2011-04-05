#!/usr/bin/env python

"""Renames topics in a bag such that tod_training (in SVN revision 50320)
understands what's inside. Telling tod_training's bag_dumper which topic name
corresponds to what data does not work as bag_dumper is buggy. Use this tool to
preprocess bags prior to training.

This script requires /opt/ros/diamondback/stacks/ros_comm/tools/rosbag/src or
similar to be in your PYTHONPATH environment variable.
"""

import os
import rosbag
import argparse

def main():
    parser = argparse.ArgumentParser("Renames topics in a bag to make it compatible with tod_training bag_dumper")
    parser.add_argument("bag_directory")
    args = parser.parse_args()
    bags_dir = args.bag_directory
    print bags_dir
    for f in os.listdir(bags_dir):
        print "Renaming topics in %s" % f
        if f.endswith(".bag"):
            srcfn = os.path.join(bags_dir, f)
            dstfn = os.path.join(bags_dir, f + ".1")
            with rosbag.Bag(dstfn, "w") as dst:
                for topic, msg, t in rosbag.Bag(srcfn):
                    if topic == "image_color":
                        dst.write("image", msg, t)
                    if topic == "points":
                        dst.write("points2", msg, t)
                    else:
                        dst.write(topic, msg, t)
        os.remove(srcfn)
        os.rename(dstfn, srcfn)

if __name__ == "__main__":
    main()

