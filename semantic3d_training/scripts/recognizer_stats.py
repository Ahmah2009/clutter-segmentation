#!/usr/bin/env python
#
"""Collects statistics about recognition of objects on test images. Given are a
set of test images and a list of objects that are on these images (ground
truth).  It runs tod_detecting recognizer on all images and for every object
that is expected on an object, it records whether it is recognized (+1) or not
(0). This will produce the 'recognition success rate' by dividing the number of
recognized objects (true positives) by the total count of objects on all test
images. The second statistics is the number of objects that have been detected
on the image but aren't actually contained within that image (type 2 error,
false positives).

See http://en.wikipedia.org/wiki/Receiver_operating_characteristic

The list of images is given by a configuration file following the format
recognized by Python's ConfigParser package.

<image-path> = <object-name>*

for example

test/kitchen-table.png = fat_free_milk silk fluorescent_paint
"""

import sys
import os
import subprocess
import optparse
import ConfigParser
import simplerecognizer

def stats(base, tod_config, mode, truthtable, verbose=False):
    # number of recognized objects
    true_positives = 0
    false_positives = 0
    # number of recognized objects in total
    total_cnt = 0
    # loop over test images
    for image, exp_objects in truthtable.items():
        if not os.path.exists(image):
            raise Exception("File %s does not exist" % image)
        exp_objects = set(exp_objects)
        total_cnt += len(exp_objects)
        if verbose:
            print "Recognizing objects on %s ... " % image
        rec_objects = simplerecognizer.recognize(base, image, tod_config, mode, False, None)
        tp_set = exp_objects.copy() & rec_objects
        fp_set = rec_objects.copy() - exp_objects
        true_positives += len(tp_set)
        false_positives += len(fp_set)
        if verbose:
            print "Expected objects:   %s" % exp_objects
            print "Recognized objects: %s" % rec_objects
            print "True positives:     %s" % tp_set
            print "False positives:    %s" % fp_set
    if verbose:
        print ""
        print "True positives in total:  %d" % true_positives
        print "False positives in total: %d" % false_positives
        print "Total objects on images:  %d" % total_cnt
    return (true_positives, false_positives, total_cnt)

def main():
    # Parse command-line
    option_parser = optparse.OptionParser("recognizer_stats [OPTIONS]")
    option_parser.add_option("-B", "--base", dest="base", help="forwarded to tod_detecting recognizer")
    option_parser.add_option("-f", "--tod_config", dest="tod_config", help="forwarded to tod_detecting recognizer")
    option_parser.add_option("-m", "--mode", default="0", dest="mode", help="forwarded to tod_detecting recognizer")
    option_parser.add_option("-t", "--truth", dest="truth", help="file that lists test images and expected objects")
    option_parser.add_option("-v", "--verbose", action="store_true", default=False, dest="verbose", help="be verbose")
    options, args = option_parser.parse_args()
    options, args = option_parser.parse_args()
    # read in truth
    cfg = ConfigParser.ConfigParser()
    cfg.read(options.truth)
    truthtable = dict()
    for key, val in cfg.items("default"):
        truthtable[key] = val.split(" ")
    # collect statistics
    s = stats(options.base, options.tod_config, options.mode, truthtable, options.verbose)
    print s

if __name__ == "__main__":
    main()

