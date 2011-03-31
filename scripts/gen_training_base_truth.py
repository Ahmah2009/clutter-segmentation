#!/usr/bin/env python
#
# TODO: rename truth to testConfig
# TODO: obsolete? not used at the moment.
#

import sys
import optparse
import ConfigParser
import os
import glob

# Parse command-line
option_parser = optparse.OptionParser("gen_directory_truth [OPTIONS] <truth-file>")
option_parser.add_option("-d", "--dir", dest="dir", help="directory containing some test pictures")
options, args = option_parser.parse_args()

cfg = ConfigParser.ConfigParser()
cfg.add_section("images")
config_txt = open(os.path.join(options.base, "config.txt")).read().strip()
subjects = set()
for line in config_txt.split("\n"):
    if line.strip() != "":
        subjects.add(line.strip())
print subjects
for subj in subjects:
    for img in glob.glob(options.base + "/" + subj + "/*.png"):
        if not img.endswith(".mask.png"):
            print img
            cfg.set("images", img, subj) 

cfg.write(open(args[0], "w"))

