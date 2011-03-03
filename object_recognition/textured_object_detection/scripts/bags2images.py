import os, glob, sys

if len(sys.argv) < 3:
  print "Usage: python scripts/bags2images.py <path to bags> <path to images>"
  exit(0)

files = glob.glob(sys.argv[1] + "/*.bag");
for file in files:
  print "Processing file" + file + "...";
  
  os.system("python scripts/bag2images.py " + file + " " + sys.argv[2] + "/" + os.path.basename(file)[:-4]);
  print("done\n");
