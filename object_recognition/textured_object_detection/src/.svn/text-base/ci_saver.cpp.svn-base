#include <ros/ros.h>
#include "textured_object_detection/shared_functions.h"

using namespace std;
using namespace sensor_msgs;

class CameraInfoSaver
{
public:
  CameraInfoSaver(const ros::NodeHandle &nh);
  void init();
private:
  void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& ci);

  ros::NodeHandle nh;
  string directory;
  ros::Subscriber ci_sub;
};

CameraInfoSaver::CameraInfoSaver(const ros::NodeHandle &_nh) :
  nh(_nh)
  {}

void CameraInfoSaver::init()
{
    directory = nh.resolveName("dir");
    string topic = nh.resolveName("ci");
    ci_sub = nh.subscribe(topic, 1, &CameraInfoSaver::cameraInfoCallback, this);
}

void CameraInfoSaver::cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& ci)
{
      createDirIsNeeded(directory);
      string filename = directory + "/info.txt";
      saveCameraInfo(ci, filename);
      ros::shutdown();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "camera_info_saver");
  ros::NodeHandle nh;

  CameraInfoSaver saver(nh);
  saver.init();

  ros::spin();
  return 0;
}
