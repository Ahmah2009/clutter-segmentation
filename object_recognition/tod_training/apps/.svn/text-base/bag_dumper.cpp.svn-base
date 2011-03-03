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
  std::string bag_file;
  std::string object_name;
  std::string base_path;

  std::string topic_points2, topic_image, topic_camera;
  fs::path full_path;
};

struct ImagePointsCamera
{
  sensor_msgs::ImageConstPtr img;
  sensor_msgs::CameraInfoConstPtr camera_info;
  sensor_msgs::PointCloud2ConstPtr points2;
  bool full() const
  {
    return (img != NULL && camera_info != NULL && points2 != NULL);
  }
  void clear()
  {
    img.reset();
    camera_info.reset();
    points2.reset();
  }
};

int options(int ac, char ** av, Options& opts)
{
  // Declare the supported options.
  po::options_description desc("Allowed options");
  desc.add_options()("help", "Produce help message.");
  desc.add_options()("bag,B", po::value<std::string>(&opts.bag_file),
                     "The bag file with Image, PointClouds2, and CameraInfo messages. Required.");
  desc.add_options()("object_name,N", po::value<std::string>(&opts.object_name),
                     "The unique name of the object, must be a valid directory name, please no spaces. Required.");
  desc.add_options()("base_path,P", po::value<std::string>(&opts.base_path),
                     "The absolute path of the training base. The object will be stored here <base_path>/<object_name> Required.");

  desc.add_options()("image", po::value<std::string>(&opts.topic_image)->default_value("image"),
                     "image topic");
  desc.add_options()("points2", po::value<std::string>(&opts.topic_points2)->default_value("points2"),
                     "points2 topic");
  desc.add_options()("camera_info", po::value<std::string>(&opts.topic_camera)->default_value("camera_info"),
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

  if (!vm.count("object_name"))
  {
    cout << "Must supply an object_name" << "\n";
    cout << desc << endl;
    return 1;
  }
  if (!vm.count("base_path"))
  {
    cout << "Must supply an base_path" << "\n";
    cout << desc << endl;
    return 1;
  }
  opts.full_path = opts.full_path / opts.base_path / opts.object_name;
  cout << "created path: " << opts.full_path.string() << endl;
  fs::create_directories(opts.full_path);
  if (!fs::exists(opts.full_path))
  {
    cerr << "could not create : " << opts.full_path << endl;
    return 1;
  }

  return 0;

}

class TodProcessor
{
public:
  TodProcessor(Options opts) :
    n(0), saved_camera_info(false), opts(opts)
  {
  }
  void process(const ImagePointsCamera& ipc)
  {
    //do cool stuff with images points and camera...
    cout << "processing.." << endl;
    {
      //write point cloud to file
      pcl::PointCloud<pcl::PointXYZRGB> cloud;
      // Convert to the templated message type
      pcl::fromROSMsg(*ipc.points2, cloud);
      pcl::io::savePCDFileBinary((opts.full_path / str(boost::format("cloud_%05d.pcd") % n)).string(), cloud);
    }
    {
      //write image to file
      const cv::Mat cv_image = bridge_.imgMsgToCv(ipc.img, "bgr8");
      cv::imwrite((opts.full_path / str(boost::format("image_%05d.png") % n)).string(), cv_image);
    }
    if (!saved_camera_info)
    {
      //write camera_info to file - assume that this is constant for the bag
      tod::Camera camera = tod::fromRosMsg(ipc.camera_info);
      fs::path camera_path = (opts.full_path / "camera.yml");
      cout << "writing camera to " << camera_path << endl;
      FileStorage fs(camera_path.string(), FileStorage::WRITE);
      fs << tod::Camera::YAML_NODE_NAME;
      camera.write(fs);
      saved_camera_info = true;
    }
    n++;

  }
  int n;
  bool saved_camera_info;
  Options opts;
  sensor_msgs::CvBridge bridge_;
};

}

int main(int argc, char ** argv)
{

  Options opts;
  if (options(argc, argv, opts))
    return 1;

  rosbag::Bag bag;
  bag.open(opts.bag_file, rosbag::bagmode::Read);

  std::vector<std::string> topics;
  topics.push_back(opts.topic_image);
  topics.push_back(opts.topic_camera);
  topics.push_back(opts.topic_points2);

  rosbag::View view(bag, rosbag::TopicQuery(topics));

  ImagePointsCamera ipc_package;

  TodProcessor p(opts);
  foreach(rosbag::MessageInstance const m, view)
        {
          sensor_msgs::ImageConstPtr img = m.instantiate<sensor_msgs::Image> ();
          if (img != NULL)
          {
            ipc_package.img = img;
          }
          sensor_msgs::CameraInfoConstPtr camera_info = m.instantiate<sensor_msgs::CameraInfo> ();
          if (camera_info != NULL)
          {
            ipc_package.camera_info = camera_info;
          }
          sensor_msgs::PointCloud2ConstPtr points2 = m.instantiate<sensor_msgs::PointCloud2> ();
          if (points2 != NULL)
          {
            ipc_package.points2 = points2;
          }

          if (ipc_package.full())
          {
            p.process(ipc_package);
            ipc_package.clear();
          }

        }

  bag.close();
}
