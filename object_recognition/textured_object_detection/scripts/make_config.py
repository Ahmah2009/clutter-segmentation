# -*- coding: utf-8 -*-
import sys, os, glob

if len(sys.argv) != 2:
  print "Usage: python make_config.py <path to train folder>"

objects = glob.glob(sys.argv[1] + "/*")
content = ""
for object_path in objects:
  if os.path.isdir(object_path) == False:
    continue
  object = os.path.basename(object_path)
  count = len(glob.glob(object_path + "/*.kpt"))
  content += object + " " + str(count) + "\n"

fp = open(sys.argv[1] + "/config.txt", "w")
fp.write(content[:-1])
fp.close()
