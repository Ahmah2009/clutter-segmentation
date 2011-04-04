#!/usr/bin/env python
"""Collects some statistics on a test description file. Might be useful to
check a test description file for typos or count the number of images, objects
and the number of different subjects seen on these images."""

import optparse
import ConfigParser
import sys
import os

def main():
    option_parser = optparse.OptionParser("testdesc-stats <testdesc-file>")
    options, args = option_parser.parse_args()
    if len(args) == 0:
        print option_parser.usage
        sys.exit(1)
    cfg = ConfigParser.ConfigParser()
    cfg.read(args[0])
    objects = set() 
    images = set()
    counts = dict()
    total_obj_cnt = 0
    for key, val in cfg.items("images"):
        images.add(key)
        val = val.strip()
        if val != "":
            for o in val.strip().split(" "):
                objects.add(o)
                total_obj_cnt += 1
                if o in counts:
                    counts[o] = counts[o] + 1
                else:
                    counts[o] = 1
    print "Set of objects on test images:"
    for o in objects:
        print "    %s" % o
    print ""
    print ""
    print "Number of appearances:"
    for o in counts:
        print "%25s: %d" % (o, counts[o])
    print ""
    print ""
    print "Number of test images: %d" % len(images) 
    print "Overall number of objects: %d" % total_obj_cnt
    print "Number of training subjects shown on any image: %d" % len(objects) 
    print ""
    print ""
    p = os.getenv("CLUTSEG_PATH")
    if p == None:
        print "CLUTSEG_PATH is not defined."
        sys.exit(1)
    else:
        bagsdir = os.path.join(p, "tod_kinect_bags")
        for o in objects:
            if not os.path.exists(os.path.join(bagsdir, "%s.bag" % o)):
                print "WARNING: no bag file found for %s" % o
            if not os.path.exists(os.path.join(bagsdir, "%s.tf.bag" % o)):
                print "WARNING: no tf bag file found for %s" % o

if __name__ == "__main__":
    main()
