# -*- coding: utf-8 -*-
import os, sys, shutil, collections

initdir = "./"
resultdir = "/home/alex/tod_bases/images3"
have_textured = 1
print len(sys.argv)
if (len(sys.argv) > 2):
     initdir = sys.argv[1]
     resultdir = sys.argv[2]
if (len(sys.argv) > 3):
     have_textured = int(sys.argv[3])
if (not(os.path.exists(resultdir))):
    os.mkdir(resultdir)

dir = initdir
ldir = os.path.join(initdir, "left")
rdir = os.path.join(initdir, "right")
ltdir = os.path.join(initdir, "left_tex")
rtdir = os.path.join(initdir, "right_tex")


rldir = os.path.join(resultdir, "left")
rrdir = os.path.join(resultdir, "right")
rltdir = os.path.join(resultdir, "left_tex")
rrtdir = os.path.join(resultdir, "right_tex")
rtestdir = os.path.join(resultdir, "test")
if (not(os.path.exists(rldir))):
    os.mkdir(rldir)
if (not(os.path.exists(rrdir))):
    os.mkdir(rrdir)
if (have_textured):
  if (not(os.path.exists(rltdir))):
    os.mkdir(rltdir)
  if (not(os.path.exists(rrtdir))):
    os.mkdir(rrtdir)

if (not(os.path.exists(rtestdir))):
    os.mkdir(rtestdir)

shutil.copyfile(os.path.join(initdir,"left_info.txt") , os.path.join(resultdir, "left_info.txt"))
shutil.copyfile(os.path.join(initdir,"right_info.txt") , os.path.join(resultdir, "right_info.txt"))
shutil.copyfile(os.path.join(initdir,"left_info.txt") , os.path.join(rtestdir, "info.txt"))

index = 1
testIndex = 1
i = 1
while (1):
   imgName = str(i) + ".png"   
   if (not(os.path.exists(os.path.join(ldir,imgName)))):
     break
   print(i)
   
   if (i%2 == 0):
     rImageName = str(index) + ".png";
     shutil.copyfile(os.path.join(ldir,imgName) , os.path.join(rldir, rImageName))
     shutil.copyfile(os.path.join(rdir,imgName) , os.path.join(rrdir, rImageName))
     if (have_textured):
       shutil.copyfile(os.path.join(ltdir,imgName) , os.path.join(rltdir, rImageName))
       shutil.copyfile(os.path.join(rtdir,imgName) , os.path.join(rrtdir, rImageName))
     index = index+1
   else:
     if (index > 100 and testIndex <=200):
       shutil.copyfile(os.path.join(ldir,imgName) , os.path.join(rtestdir, str(testIndex)+".png"))
       testIndex = testIndex+1
   i = i+1
