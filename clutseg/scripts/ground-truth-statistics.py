#!/usr/bin/env python

import optparse
import ConfigParser
import sys
import os

def main():
    option_parser = optparse.OptionParser("ground-truth-statistics <testdesc-file>")
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
    print "Number of different objects shown: %d" % len(objects) 

if __name__ == "__main__":
    main()
