#!/usr/bin/env python
#
# Wraps recognizer in tod_detecting due to its lack of any sensible
# output formatting.
#

import sys
import os
import subprocess
import optparse

def recognize(base, image, tod_config, mode, bestonly, expect, verbose=False):
    desc = ("rosrun", "tod_detecting", "recognizer",
            "--verbose=0",
            "-B", base,
            "-I", image,
            "-f", tod_config,
            "-m", mode)

    # Find out what objects exist in the database
    config_txt = open(os.path.join(base, "config.txt")).read().strip()
    objects = set()
    for line in config_txt.split("\n"):
        if line.strip() != "":
            objects.add(line.strip())

    process = subprocess.Popen(desc, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    stdout, stderr = process.communicate()

    # The best match is the object that appears most
    # often in the program output reduced to the lines
    # that contain "Object name". Of course this is a
    # heuristic and might be adapted to whatever SVN
    # revision of tod_detecting we're using.
    recog = ""
    for line in stdout.split("\n"):
        if "Object name" in line:
            recog += line

    best_match_obj = None
    best_match_cnt = 0
    rec_objects = set()
    for obj in objects:
        c = recog.count(obj)
        if not bestonly and c >= 1:
            if verbose:
                print obj
            rec_objects.add(obj)
        if c > best_match_cnt:
            best_match_cnt = c
            best_match_obj = obj

    if bestonly and verbose:
        print best_match_obj

    if expect:
        if best_match_obj == expect:
            sys.exit(0)
        else:
            sys.exit(1)
    elif bestonly:
        return best_match_obj
    else:
        return rec_objects

def main():
    # Parse command-line
    option_parser = optparse.OptionParser("simplerecognizer [OPTIONS]")
    option_parser.add_option("-B", "--base", dest="base", help="forwarded to tod_detecting recognizer")
    option_parser.add_option("-I", "--image", dest="image", help="forwarded to tod_detecting recognizer")
    option_parser.add_option("-f", "--tod_config", dest="tod_config", help="forwarded to tod_detecting recognizer")
    option_parser.add_option("-m", "--mode", default="0", dest="mode", help="forwarded to tod_detecting recognizer")
    option_parser.add_option("-b", "--bestonly", default=True, dest="bestonly", help="whether to print only the object that is most likely on the test image")
    option_parser.add_option("-e", "--expect", dest="expect", help="return 0 if this object has been recognized on the picture, 1 otherwise")
    options, args = option_parser.parse_args()
    # Run recognition process
    recognize(options.base, options.image, options.tod_config, options.mode, options.bestonly, options.expect, True)

if __name__ == "__main__":
    main()

