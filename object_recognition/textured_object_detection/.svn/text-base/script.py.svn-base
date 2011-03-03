# -*- coding: utf-8 -*-
import os, sys, shutil
indexes = [[...]]
names = ["..."]
testdir = "..."
dst = "./"

writeInfo = False
ind = 0
for name in names:
  srcdir = os.path.join(testdir, name)
  dstdir = os.path.join(dst, name)
  if (not(os.path.exists(dstdir))):
    os.mkdir(dstdir)

  imgFolder = os.path.join(srcdir, "obj")
  objFolder = os.path.join(srcdir, "obj_cloud")

  camparam = "info.txt"

  fh = open(os.path.join(srcdir,camparam), "r")
  content = fh.read()
  #content = content.replace(",", ".")
  #content = content.replace("\t", " ")
  #content = content.replace("\n", " ")
  fh.close()

  f = open(os.path.join(dstdir, camparam),"w")
  f.write(content)

  if (not(writeInfo)):
    f = open(os.path.join(dst, camparam),"w")
    f.write(content)
    writeInfo = True  

  i = 0;
  for index in indexes[ind]:
    i = i + 1
    print(index)
    fileIndex = str(index)
    if (index < 10):
      fileIndex = "0" + fileIndex;
    imgName = "object" + fileIndex + ".png"
    shutil.copyfile(os.path.join(imgFolder,imgName) , os.path.join(dstdir, str(i) + ".png"))
    objName = "object_cloud" + fileIndex + ".pcd"
    shutil.copyfile(os.path.join(objFolder,objName) , os.path.join(dstdir, str(i) + ".pcd"))
    print(imgName)
    print(objName)
  
  ind = ind + 1
