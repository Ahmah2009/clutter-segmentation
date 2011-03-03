/*
 * cisaver.cpp
 *
 *  Created on: Feb 19, 2011
 *      Author: Alexander Shishkov
 */
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>

#include <boost/program_options.hpp>
#include <boost/foreach.hpp>
#include <boost/filesystem.hpp>
#include "boost/format.hpp"

#include <iostream>
#include <string>

#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"

#include <tod/training/ros/msgs.h>
#include <tod/core/Camera.h>

#include <cv_bridge/CvBridge.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#define foreach         BOOST_FOREACH
#define reverse_foreach BOOST_REVERSE_FOREACH

namespace po = boost::program_options;
namespace fs = boost::filesystem;

using std::cout;
using std::endl;
using std::cerr;

using namespace cv;

namespace
{
struct Options
{
  std::string bagFile;
  std::string filePath;
  std::string topicCamera;
};

int options(int ac, char ** av, Options& opts)
{
  // Declare the supported options.
  po::options_description desc("Allowed options");
  desc.add_options()("help", "Produce help message.");
  desc.add_options()("bag,b", po::value<std::string>(&opts.bagFile),
                     "The bag file CameraInfo messages. Required.");
  desc.add_options()("file_path,f", po::value<std::string>(&opts.filePath),
                     "The absolute path of the training base. The object will be stored here <base_path>/<object_name> Required.");

  desc.add_options()("camera_info,c", po::value<std::string>(&opts.topicCamera)->default_value("camera_info"),
                     "camera_info topic");

  po::variables_map vm;
  po::store(po::parse_command_line(ac, av, desc), vm);
  po::notify(vm);

  if (!vm.count("bag"))
  {
    cout << "Must supply a bag file name" << "\n";
    cout << desc << endl;
    return 1;
  }

  if (!vm.count("file_path"))
  {
    cout << "Must supply an result filepath" << "\n";
    cout << desc << endl;
    return 1;
  }
  return 0;
}

class CISaver
{
public:
  CISaver(Options opts): opts(opts)
  {
  }

  void process(sensor_msgs::CameraInfoConstPtr cameraInfo)
  {
    //write camera_info to file - assume that this is constant for the bag
    tod::Camera camera = tod::fromRosMsg(cameraInfo);
    cout << "writing camera to " << opts.filePath << endl;
    FileStorage fs(opts.filePath, FileStorage::WRITE);
    fs << tod::Camera::YAML_NODE_NAME;
    camera.write(fs);
  }
  Options opts;
};
}

int main(int argc, char ** argv)
{

  Options opts;
  if (options(argc, argv, opts))
    return 1;

  rosbag::Bag bag;
  bag.open(opts.bagFile, rosbag::bagmode::Read);

  std::vector<std::string> topics;
  topics.push_back(opts.topicCamera);

  rosbag::View view(bag, rosbag::TopicQuery(topics));

  CISaver saver(opts);
  foreach(rosbag::MessageInstance const message, view)
  {
    sensor_msgs::CameraInfoConstPtr cameraInfo = message.instantiate<sensor_msgs::CameraInfo> ();
    if (cameraInfo != NULL)
    {
      saver.process(cameraInfo);
      break;
    }
  }
  bag.close();
}

