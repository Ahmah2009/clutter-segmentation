#!/usr/bin/env python
#
# Wraps recognizer in tod_detecting due to its lack of any sensible
# output formatting.
#

import sys
import os
import subprocess
import optparse

option_parser = optparse.OptionParser("simplerecognizer [OPTIONS]")
option_parser.add_option("-B", "--base", dest="base")
option_parser.add_option("-I", "--image", dest="image")
option_parser.add_option("-f", "--tod_config", dest="tod_config")
option_parser.add_option("-m", "--mode", default="0", dest="mode")
option_parser.add_option("-e", "--expect", dest="expect", help="return 0 if this object has been recognized on the picture, 1 otherwise")
options, args = option_parser.parse_args()

desc = ("rosrun", "tod_detecting", "recognizer",
        "--verbose=0",
        "-B", options.base,
        "-I", options.image,
        "-f", options.tod_config,
        "-m", options.mode)

# Find out what objects exist in the database
config_txt = open(os.path.join(options.base, "config.txt")).read().strip()
objects = []
for line in config_txt.split("\n"):
    if line.strip() != "":
        objects.append(line.strip())

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
for obj in objects:
    c = recog.count(obj)
    if c > best_match_cnt:
        best_match_cnt = c
        best_match_obj = obj

# Print/return result of recognition process
print best_match_obj
if options.expect:
    if best_match_obj == options.expect:
        sys.exit(0)
    else:
        sys.exit(1)
