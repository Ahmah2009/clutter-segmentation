# -*- coding: utf-8 -*-
import os, glob, sys

if len(sys.argv) < 5:
  print "Usage: python scripts/testsystem.py <training base> <test_base> <configuration file> <log file>"
  exit(0)

objects = glob.glob(sys.argv[2] + "/*")
for object_path in objects:
  if os.path.isdir(object_path) == False:
    continue
  print "Processing directory " + object_path + "...";
  os.system("./bin/rtc -ci ./camera/pro_info.txt -config " + sys.argv[3] + " -log " + sys.argv[4] + " " + sys.argv[1] + " " + object_path)
  print("done\n")
