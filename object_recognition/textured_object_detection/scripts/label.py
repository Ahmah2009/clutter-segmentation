# -*- coding: utf-8 -*-
import os, sys, glob

if len(sys.argv) < 5:
  print "Usage: python scripts/label.py <path to source data> <path to ouptut train folder> <config file> [-label | -process]\n"
  exit(0);

paths = glob.glob(sys.argv[1] + "/*")
mode = "-mask-only" if sys.argv[4] == "-label" else ""
count = 0;
for path in paths:
  object_name = os.path.basename(path)
  sys.stdout.write("Process " + object_name + "...")
  sys.stdout.flush()

  os.system("bin/crop_auto " + path + " " + object_name + " " + sys.argv[2] + " -transform camera/transform.xml -config " + sys.argv[3] + " " + mode);

  sys.stdout.write("done, %d%% objects labeled\n" % (float(++count)/float(len(paths))*100))
  count = count + 1;

