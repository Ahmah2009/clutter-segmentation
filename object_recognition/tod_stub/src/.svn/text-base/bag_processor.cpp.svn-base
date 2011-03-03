#include <rosbag/bag.h>
#include <rosbag/view.h>


#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>


#include <boost/program_options.hpp>
#include <boost/foreach.hpp>

#include <iostream>
#include <string>

#define foreach         BOOST_FOREACH
#define reverse_foreach BOOST_REVERSE_FOREACH

namespace po = boost::program_options;

using std::cout;
using std::endl;

namespace
{
struct Options
{
  std::string bag_file;
};

struct ImagePointsCamera
{
  sensor_msgs::ImageConstPtr img;
  sensor_msgs::CameraInfoConstPtr camera_info;
  sensor_msgs::PointCloud2ConstPtr points2;
  bool full() const{
    return (img != NULL && camera_info != NULL && points2 != NULL);
  }
  void clear(){
    img.reset();
    camera_info.reset();
    points2.reset();
  }
};

int options(int ac, char ** av, Options& opts)
{
  // Declare the supported options.
  po::options_description desc("Allowed options");
  desc.add_options()("help", "Produce help message.")
      ("bag,B", po::value<std::string>(&opts.bag_file),
        "The bag file with Image, PointClouds2, and CameraInfo messages. Required.")
  ;


  po::variables_map vm;
  po::store(po::parse_command_line(ac, av, desc), vm);
  po::notify(vm);

  if (!vm.count("bag"))
  {
    cout << "Must supply a bag file name" << "\n";
    cout << desc << endl;
    return 1;
  }
  return 0;

}

void process(const ImagePointsCamera& ipc){
  //do cool stuff with images points and camera...
  cout << "processing.."<<endl;
}

}

int main(int argc, char ** argv)
{

  Options opts;
  if (options(argc, argv, opts))
    return 1;

  rosbag::Bag bag;
  bag.open(opts.bag_file, rosbag::bagmode::Read);

  std::vector<std::string> topics;
  topics.push_back("image");
  topics.push_back("camera_info");
  topics.push_back("points2");

  rosbag::View view(bag, rosbag::TopicQuery(topics));

  ImagePointsCamera ipc_package;
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

    if(ipc_package.full()){
      process(ipc_package);
      ipc_package.clear();
    }

  }

  bag.close();
}
