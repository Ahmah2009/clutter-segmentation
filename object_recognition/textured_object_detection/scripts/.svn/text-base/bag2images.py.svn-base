import sys, os, signal, commands, time

pid1 = os.spawnlp(os.P_NOWAIT, "roslaunch", "roslaunch", "textured_object_detection", "image_sub.launch", "directory:=" + sys.argv[2]);
#environment = os.environ;
#environment["ROS_NAMESPACE"] = "/prosilica";
#pid2 = os.spawnlpe(os.P_NOWAIT, "rosrun", "rosrun", "image_proc", "image_proc", environment);
os.system("rosbag play " + sys.argv[1] + " -r2");

pidis = commands.getoutput('pidof is').split(' ')
pidimgproc = commands.getoutput('pidof image_proc').split(' ')

os.kill(pid1, signal.SIGKILL)

for p in pidis:
  os.kill(int(p), signal.SIGKILL)
  #os.waitpid(int(p), 0)
  time.sleep(5)
for p in pidimgproc:
  os.kill(int(p), signal.SIGKILL)
  #os.waitpid(int(p), 0)
  time.sleep(5)

os.waitpid(pid1, 0)

