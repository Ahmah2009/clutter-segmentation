#!/usr/bin/env python
import os
import Queue
import subprocess
import sys
import threading

class BagDumper(threading.Thread):
  def __init__(self, argument_queue):
    threading.Thread.__init__(self)
    self.argument_queue = argument_queue
  
  def run(self):
    while True:
      try:
        argument = argument_queue.get(False, 1)
      except Queue.Empty:
        break
      process = subprocess.Popen('rosrun tod_training bag_dumper -P %s -N %s -B %s' % argument, shell=True)
      stdout, stderr = process.communicate()
      #print stdout
      if stderr:
        print stderr

if __name__ == '__main__':
  if(len(sys.argv) != 3):
      print """usage:
      dump_all <path_to_bags> <path_to_base>"""
      sys.exit(1)
  argument_queue = Queue.Queue()
  for file in os.listdir(sys.argv[1]):
    if 'tf' not in file and file.endswith('.bag'):
      objectName = '_'.join(file.split('.')[:-1])
      argument_queue.put((sys.argv[2], objectName, os.path.join(sys.argv[1], file)))
  
  thread_list = []
  #for _i in xrange(int(sys.argv[3])):
  thread_list.append(BagDumper(argument_queue))
  thread_list[-1].daemon = True
  thread_list[-1].start()
  
  while thread_list:
    thread_list[0].join()
    thread_list.pop(0)
