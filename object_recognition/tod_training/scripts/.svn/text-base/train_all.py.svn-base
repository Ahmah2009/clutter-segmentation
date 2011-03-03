#!/usr/bin/env python
import os
import Queue
import subprocess
import sys
import threading

from optparse import OptionParser

pose_estimator = "pose_estimator -d %s -j %d %s"
pose_estimator = "masker -d %s -j %d %s"
pose_estimator = "detector -d %s -j %d %s"

class Trainer():
    """
    rosrun tod_training pose_estimator -d $1 -j$JOBS
    rosrun tod_training masker -d $1 -M 1 -j$JOBS
    #0 is for 3d box, #1 is for pcl segmentation
    rosrun tod_training detector -d $1 -j$JOBS
    rosrun tod_training f3d_creator -d $1 -j$JOBS
    """
    
    def __init__(self, options, args):
        self.options=options
        self.args=args
  
    
        
    def run_process(self,command,arguments):
        command = command % argument
        command = "rosrun tod_training %s" % command
        process = subprocess.Popen(command, shell=True)
        stdout, stderr = process.communicate()
        if stderr:
            print stderr

def pose_estimator(base_dir,file):
    process = subprocess.Popen('rosrun tod_training tod-P %s -N %s -B %s' % argument, shell=True)
    stdout, stderr = process.communicate()
    if stderr:
        print stderr
if __name__ == '__main__':
    parser = OptionParser(description='Train all objects in a training base.')
    parser.add_option("-B", "--base_dir", dest="base_dir",
                  help="the base directory, where all my objects are dumped", metavar="PATH_TO_BASE",
                   default='./')
    parser.add_option("-q", "--quiet",
                  action="store_false", dest="verbose", default=True,
                  help="don't print status messages to stdout")

    (options, args) = parser.parse_args()
    if not options.base_dir:
        print parser.h
        sys.exit(1)
    print "base_dir is %s" % options.base_dir
    for file in os.listdir(options.base_dir):
        print "training %s" %file
        process = subprocess.Popen('rosrun tod_training  -P %s -N %s -B %s' % argument, shell=True)
        stdout, stderr = process.communicate()
        if stderr:
            print stderr



