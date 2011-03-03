# -*- coding: utf-8 -*-
#!/usr/bin/env python
import os
import Queue
import subprocess
import sys
import threading
import glob

from optparse import OptionParser

def run_process(command):
    process = subprocess.Popen(command, shell=True)
    stdout, stderr = process.communicate()
    if stderr:
        print stder

def create_dir(directory):
    if not os.path.exists(directory):
      process = subprocess.Popen('mkdir %s' % directory, shell=True)

if __name__ == '__main__':
    parser = OptionParser(description='Create test base using bag file with test data')
    parser.add_option("-B", "--base_dir", dest="base_dir",
                  help="the test directory", metavar="PATH_TO_BASE")
    parser.add_option("-b", "--bag_file", dest="bag_file",
                  help="the bag file path", metavar="PATH_TO_BAG")
    parser.add_option("-f", "--fiducial", dest="fiducial_path",
                  help="fiducial.yaml path", default='fiducial.yaml')
    parser.add_option("-c", "--count", dest="count", default=1, help="count of the containing objects")
    parser.add_option("-q", "--quiet", action="store_false", dest="verbose", default=True, help="don't print status messages to stdout")

    (options, args) = parser.parse_args()
    if not options.base_dir or not options.bag_file:
        parser.print_help()
        sys.exit(1)

    print "base_dir is %s" % options.base_dir
    print "test_bag_file is %s" % options.bag_file

    create_dir(options.base_dir) 
    
    dump_dir = 'dump'
    img_dir = 'images'
    bag_file_name = os.path.basename(options.bag_file)[:-4]
    bag_file_dir = os.path.join(options.base_dir, bag_file_name)
    #run_process('rosrun tod_training bag_dumper -P %s -N %s -B %s' % (bag_file_dir, dump_dir, options.bag_file))
    create_dir(bag_file_dir) 
    create_dir(os.path.join(bag_file_dir, dump_dir)) 
    from grab_images_from_bag import grab 
    grab(options.bag_file, os.path.join(bag_file_dir, dump_dir))

    run_process('rosrun tod_test cisaver -b %s -f %s -c /camera/rgb/camera_info' % (options.bag_file, os.path.join(bag_file_dir, dump_dir, 'camera.yml')))
    
    print('Calculating poses...') 
    run_process('rosrun tod_training pose_estimator -d %s -F %s -j 8' % (os.path.join(bag_file_dir, dump_dir), options.fiducial_path))
    
    create_dir(os.path.join(bag_file_dir, img_dir))

    poselist = glob.glob('%s/*.pose.yaml' %  (os.path.join(bag_file_dir, dump_dir)))
    for posefile in poselist:
      run_process('cp %s %s' % (posefile[:-len('.pose.yaml')], os.path.join(bag_file_dir, img_dir)))
      
    for i in range(int(options.count)):
      print "type object name:"
      name = sys.stdin.readline()
      print "training %s" % name 
      run_process('rosrun tod_training masker -M 0 -d %s -j 8' % (os.path.join(bag_file_dir, dump_dir)))  
      create_dir(os.path.join(bag_file_dir, name))
      masklist = glob.glob('%s/*.mask.png' %  (os.path.join(bag_file_dir, dump_dir)))
      for mask in masklist:
        run_process('mv %s %s' % (mask, os.path.join(bag_file_dir, name)))
           
    
