#!/usr/bin/env python

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
        if f.endswith(".bag"):
            print "Renaming topics in %s" % f
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

