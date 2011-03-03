#include <tod_stub/tod_stub.h>
#include <cv_bridge/CvBridge.h>
#include <pcl/io/pcd_io.h>
#include <pcl/ros/conversions.h>
#include <pcl_ros/point_cloud.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

namespace fs = boost::filesystem;
namespace po = boost::program_options;

namespace tod_stub
{
bool ImagePointsCamera::full() const
{
  return (img != NULL && camera_info != NULL && points2 != NULL);
}
void ImagePointsCamera::clear()
{
  img.reset();
  camera_info.reset();
  points2.reset();
}

void ImagePointsCamera::convert(FrameData& frame) const
{
  sensor_msgs::CvBridge bridge;
  //write point cloud to file
  Cloud_t cloud;
  // Convert to the templated message type
  frame.cloud = *points2;

  //write image to file
  const cv::Mat cv_image = bridge.imgMsgToCv(img, "bgr8");
  cv_image.copyTo(frame.image);

  frame.D = cv::Mat(camera_info->D).clone();
  frame.K = cv::Mat(3, 3, CV_64F, (void*)camera_info->K.elems).clone();
}

void ImagePointsCamera::ProcessBag(const rosbag::Bag& bag, ProcessorCallable processor,
                                   const std::vector<std::string>& _topics)
{
  std::vector<std::string> topics = _topics;
  if (topics.empty())
  {
    topics.push_back("image");
    topics.push_back("camera_info");
    topics.push_back("points2");
  }
  rosbag::View view(bag, rosbag::TopicQuery(topics));

  ImagePointsCamera ipc_package;
  BOOST_FOREACH(rosbag::MessageInstance const m, view)
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
          Cloud_t::ConstPtr points2 = m.instantiate<Cloud_t> ();
          if (points2 != NULL)
          {
            ipc_package.points2 = points2;
          }

          if (ipc_package.full())
          {
            try
            {
              processor(ipc_package);
            }
            catch (tod_exception e)
            {
              switch (e)
              {
                case tod_stub::QUIT:
                  std::cout << "quit" << std::endl;
                  return;
                  break;
                case tod_stub::BAD_POSE:
                  break;
                case tod_stub::ERROR:
                  //std::cerr << "error" << std::endl;
                  break;
              }
            }
            ipc_package.clear();
          }
        }

}

namespace
{

struct MyProcessor : public ImagePointsCamera::Processor
{

  void operator()(const ImagePointsCamera& ipc)
  {
    //do cool stuff with images points and camera...
    cout << "processing.. interface" << endl;
  }
};

struct MyGProcessor
{

  void operator()(const ImagePointsCamera& ipc)
  {
    //do cool stuff with images points and camera...
    cout << "processing.. generic" << endl;
  }
};

void process(const ImagePointsCamera& ipc)
{
  //do cool stuff with images points and camera...
  cout << "processing.. function pointer" << endl;
}

void instantiate_test()
{
  MyProcessor p;
  MyGProcessor g;
  rosbag::Bag bag;
  ImagePointsCamera::ProcessBag(bag, p);
  ImagePointsCamera::ProcessBag(bag, process);
  ImagePointsCamera::ProcessBag(bag, g);
}
}

namespace po = boost::program_options;

int options(int ac, char ** av, Options& opts)
{
  // Declare the supported options.
  po::options_description desc = opts.desc;
  desc.add_options()("help", "Produce help message.");
  desc.add_options()("bag,B", po::value<std::string>(&opts.bag_file),
                     "The bag file with Image, PointClouds2, and CameraInfo messages.");
  desc.add_options()("config_file,C", po::value<std::string>(&opts.config_file)->default_value("user_config.yml"),
                     "The user's algorithm configuration filename.");
  //desc.add_options()("tod_config,T", po::value<std::string>(&opts.tod_config_file),
  //                   "The tod specific configuration filename. This specifies the object names to train, etc...");

  po::variables_map vm;
  po::store(po::parse_command_line(ac, av, desc), vm);
  po::notify(vm);

  fs::path bag_path = opts.bag_file;

  if (vm.count("help"))
  {
    cout << desc << endl;
    return 1;
  }
  
  if (opts.bag_file.empty() || !fs::exists(bag_path))
  {
    std::cerr << "Must supply a valid bag file name: " << bag_path.string() << " does not exist." << "\n";
    cout << desc << endl;
    return 1;
  }
  return 0;
}

PoseDrawer3d::PoseDrawer3d() :
  prefix_("obj"), update_(false)
{

}
void PoseDrawer3d::setRT(cv::Mat r, cv::Mat t, const std::string& prefix)
{
  boost::mutex::scoped_lock(set_mtx_);
  if (!r.empty())
    r.convertTo(r_, CV_64F);
  if (!t.empty())
  {
    t.convertTo(t_, CV_64F);
  }
  update_ = true;
  prefix_ = prefix;
}

void PoseDrawer3d::drawResults(const std::vector<Result>& results)
{
  boost::mutex::scoped_lock(set_mtx_);
  previous_results_ = results_;
  results_ = results;
  update_ = true;
}
void remove(pcl_visualization::PCLVisualizer& viewer, const std::string& prefix)
{
  std::cout << "removing " << prefix << std::endl;
  //remove old coordinate frame
  viewer.removeShape(std::string(prefix + "co_x"));
  viewer.removeShape(std::string(prefix + "co_y"));
  viewer.removeShape(std::string(prefix + "co_z"));
  viewer.removeShape(std::string(prefix + "text"));
}

void drawPose(pcl_visualization::PCLVisualizer& viewer, const std::string& prefix, cv::Mat r, cv::Mat t)
{
  std::vector<pcl::PointXYZ> coordinate_axis(4);
  if (t.empty() || r.empty())
    return;
  remove(viewer, prefix);
  //setup a canonical coordinate system
  cv::Point3d z(0, 0, 0.25);
  cv::Point3d x(0.25, 0, 0);
  cv::Point3d y(0, 0.25, 0);
  cv::Point3d o(0, 0, 0);
  std::vector<cv::Point3d> op(4);
  op[1] = x, op[2] = y, op[3] = z, op[0] = o;
  cv::Mat mop(op);
  //std::cout << r << std::endl;
  //rotate the points in place
  cv::transform(mop, mop, r);

  //translate the points and add to pcl style point vector
  int idx = 0;
  BOOST_FOREACH(cv::Point3d& p, op)
        {
          p += t.at<cv::Point3d> ();
          coordinate_axis[idx++] = pcl::PointXYZ(p.x, p.y, p.z);
        }

  viewer.addLine(coordinate_axis[0], coordinate_axis[1], 1, 0, 0, std::string(prefix + "co_x"));
  viewer.addLine(coordinate_axis[0], coordinate_axis[2], 0, 1, 0, std::string(prefix + "co_y"));
  viewer.addLine(coordinate_axis[0], coordinate_axis[3], 0, 0, 1, std::string(prefix + "co_z"));

  //viewer.addText(prefix, coordinate_axis[3].x, coordinate_axis[3].y, coordinate_axis[3].z, prefix + "text");
}

void PoseDrawer3d::operator()(pcl_visualization::PCLVisualizer& viewer)
{
  boost::mutex::scoped_lock(set_mtx_);
  //check if we have a new pose
  if (!update_)
    return;
  update_ = false;
  remove(viewer, prefix_);
  drawPose(viewer, prefix_, r_, t_);
  BOOST_FOREACH(const Result& r, previous_results_)
           remove(viewer, r.object_id);
  BOOST_FOREACH(const Result& r, results_)
  {
    //remove(viewer, r.object_id);
    drawPose(viewer, r.object_id, r.R, r.T);
    viewer.addText(r.object_id,40,40,r.object_id + "text");
  }
}

}
