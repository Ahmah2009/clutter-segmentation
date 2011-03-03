#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>
#include <cv_bridge/CvBridge.h>
#include <opencv/highgui.h>
#include <iostream>

#include "textured_object_detection/shared_functions.h"

using namespace std;
using namespace sensor_msgs;

class ImagesGenerator
{
public:
  ImagesGenerator(const ros::NodeHandle &nh);
  void init();
private:
  void leftCameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& ci);
  void rightCameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& ci);
  void proCameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& ci);
  void visualFeaturesCallback(const ImageConstPtr &imagel, const ImageConstPtr &imager);
  void visualFeaturesProCallback(const ImageConstPtr &imagel, const ImageConstPtr &imager, const ImageConstPtr& imagepro);
  void visualFeaturesExtendedCallback(const ImageConstPtr &imagel, const ImageConstPtr &imager,
                                      const ImageConstPtr &imagelt, const ImageConstPtr &imagert);
  void visualFeaturesExtendedProCallback(const ImageConstPtr &imagel, const ImageConstPtr &imager,
                                         const ImageConstPtr &imagelt, const ImageConstPtr &imagert,
                                         const ImageConstPtr& imagepro);
  void writeImage(string dir, IplImage* image);

  bool have_textured;
  bool have_prosilica;
  ros::NodeHandle nh;
  string directory;
  ros::Subscriber ci_sub_l;
  ros::Subscriber ci_sub_p;
  ros::Subscriber ci_sub_r;
  message_filters::Subscriber<Image> image_sub_l;
  message_filters::Subscriber<Image> image_sub_r;

  message_filters::Subscriber<Image> image_sub_lt;
  message_filters::Subscriber<Image> image_sub_rt;

  message_filters::Subscriber<Image> image_sub_pro;

  typedef message_filters::sync_policies::ApproximateTime<Image, Image> SyncPolicy;
  typedef message_filters::sync_policies::ApproximateTime<Image, Image, Image> ProSyncPolicy;
  typedef message_filters::sync_policies::ApproximateTime<Image, Image, Image, Image> ExtSyncPolicy;
  typedef message_filters::sync_policies::ApproximateTime<Image, Image, Image, Image, Image> ProExtSyncPolicy;

  message_filters::Synchronizer<SyncPolicy> image_sync;
  message_filters::Synchronizer<ProSyncPolicy> pro_image_sync;
  message_filters::Synchronizer<ExtSyncPolicy> ext_image_sync;
  message_filters::Synchronizer<ProExtSyncPolicy> pro_ext_image_sync;
  sensor_msgs::CvBridge bridge_l;
  sensor_msgs::CvBridge bridge_r;
  sensor_msgs::CvBridge bridge_lt;
  sensor_msgs::CvBridge bridge_rt;
  sensor_msgs::CvBridge bridge_pro;
  int file_count;
};

ImagesGenerator::ImagesGenerator(const ros::NodeHandle &_nh) :
  nh(_nh), image_sync(SyncPolicy(20)), pro_image_sync(ProSyncPolicy(40)), ext_image_sync(ExtSyncPolicy(20)), pro_ext_image_sync(ProExtSyncPolicy(20)),
      file_count(1)
{
  nh.getParam("have_textured", have_textured);
  nh.getParam("have_prosilica", have_prosilica);
}

void ImagesGenerator::init()
{
  directory = nh.resolveName("dir");
  createDirIsNeeded(directory);
  string ci_left_topic = nh.resolveName("ci_left_in");
  string ci_right_topic = nh.resolveName("ci_right_in");
  ci_sub_l = nh.subscribe(ci_left_topic, 1, &ImagesGenerator::leftCameraInfoCallback, this);
  ci_sub_r = nh.subscribe(ci_right_topic, 1, &ImagesGenerator::rightCameraInfoCallback, this);

  string image_left_topic, image_right_topic, image_left_tex_topic, image_right_tex_topic, image_prosilica_topic;
  image_left_topic = nh.resolveName("im_left_in");
  image_right_topic = nh.resolveName("im_right_in");

  int queue_size = 2000;
  image_sub_l.subscribe(nh, image_left_topic, queue_size);
  image_sub_r.subscribe(nh, image_right_topic, queue_size);

  if (have_prosilica)
  {
    string ci_pro_topic = nh.resolveName("ci_pro_in");
    ci_sub_p = nh.subscribe(ci_pro_topic, 1, &ImagesGenerator::proCameraInfoCallback, this);

    image_prosilica_topic = nh.resolveName("im_pro_in");
    image_sub_pro.subscribe(nh, image_prosilica_topic, queue_size);
  }

  if (have_textured)
  {
    image_left_tex_topic = nh.resolveName("imt_left_in");
    image_right_tex_topic = nh.resolveName("imt_right_in");
    image_sub_lt.subscribe(nh, image_left_tex_topic, queue_size);
    image_sub_rt.subscribe(nh, image_right_tex_topic, queue_size);

    if (have_prosilica)
    {
      pro_ext_image_sync.connectInput(image_sub_l, image_sub_r, image_sub_lt, image_sub_rt, image_sub_pro);
      pro_ext_image_sync.registerCallback(boost::bind(&ImagesGenerator::visualFeaturesExtendedProCallback, this, _1,
                                                      _2, _3, _4, _5));
    }
    else
    {
      pro_ext_image_sync.connectInput(image_sub_l, image_sub_r, image_sub_lt, image_sub_rt);
      pro_ext_image_sync.registerCallback(boost::bind(&ImagesGenerator::visualFeaturesExtendedCallback, this, _1, _2,
                                                      _3, _4));
    }
  }
  else
  {
    if (have_prosilica)
    {
      pro_image_sync.connectInput(image_sub_l, image_sub_r, image_sub_pro);
      pro_image_sync.registerCallback(boost::bind(&ImagesGenerator::visualFeaturesProCallback, this, _1, _2, _3));

    }
    else
    {
      image_sync.connectInput(image_sub_l, image_sub_r);
      image_sync.registerCallback(boost::bind(&ImagesGenerator::visualFeaturesCallback, this, _1, _2));
    }
  }
}

void ImagesGenerator::leftCameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& ci)
{
  string filename = directory + "/left_info.txt";
  saveCameraInfo(ci, filename);
}

void ImagesGenerator::rightCameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& ci)
{
  string filename = directory + "/right_info.txt";
  saveCameraInfo(ci, filename);
}

void ImagesGenerator::proCameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& ci)
{
  string filename = directory + "/pro_info.txt";
  saveCameraInfo(ci, filename);
}

void ImagesGenerator::writeImage(string dir, IplImage* image)
{
  mkdir(dir.c_str(), 0777);
  char image_file_name[256];
  sprintf(image_file_name, "/%d.png", file_count);
  string filename = dir + image_file_name;
  cvSaveImage(filename.c_str(), image);
  ROS_INFO_STREAM("Saving " << image_file_name);
}

// a version for prosilica together with textured stereo
void ImagesGenerator::visualFeaturesExtendedProCallback(const ImageConstPtr &imagel, const ImageConstPtr &imager,
                                                        const ImageConstPtr &imagelt, const ImageConstPtr &imagert,
                                                        const ImageConstPtr& imagepro)
{
  ROS_INFO("visualFeaturesExtendedProCallback");

  ROS_ASSERT(imagel);
  ROS_ASSERT(imager);
  ROS_ASSERT(imagelt);
  ROS_ASSERT(imagert);
  ROS_ASSERT(imagepro);

  string encoding = "passthrough";
  IplImage *cv_image_l = NULL, *cv_image_r = NULL, *cv_image_lt = NULL, *cv_image_rt = NULL, *cv_image_pro = NULL;
  try
  {
    cv_image_l = bridge_l.imgMsgToCv(imagel, encoding.c_str());
    cv_image_r = bridge_r.imgMsgToCv(imager, encoding.c_str());
    cv_image_lt = bridge_lt.imgMsgToCv(imagelt, encoding.c_str());
    cv_image_rt = bridge_rt.imgMsgToCv(imagert, encoding.c_str());
    if (imagepro->encoding.find("bayer") != std::string::npos)
      boost::const_pointer_cast<sensor_msgs::Image>(imagepro)->encoding = "mono8";
    cv_image_pro = bridge_pro.imgMsgToCv(imagepro, "bgr8");
  }
  catch (sensor_msgs::CvBridgeException error)
  {
    ROS_ERROR("BaseGenerator::visualFeaturesExtendedProCallback: Could not convert image message to IplImage");
    return;
  }

  string path, left, right, leftt, rightt, pro;

  left = "/left";
  right = "/right";
  leftt = "/left_tex";
  rightt = "/right_tex";
  pro = "/pro";

  path = directory + left;
  writeImage(path, cv_image_l);

  path = directory + right;
  writeImage(path, cv_image_r);

  path = directory + leftt;
  writeImage(path, cv_image_lt);

  path = directory + rightt;
  writeImage(path, cv_image_rt);

  path = directory + pro;
  writeImage(path, cv_image_pro);

  file_count++;
}

void ImagesGenerator::visualFeaturesExtendedCallback(const ImageConstPtr &imagel, const ImageConstPtr &imager,
                                                     const ImageConstPtr &imagelt, const ImageConstPtr &imagert)
{
  ROS_INFO("visualFeaturesExtendedCallback");

  ROS_ASSERT(imagel);
  ROS_ASSERT(imager);
  ROS_ASSERT(imagelt);
  ROS_ASSERT(imagert);

  string encoding = "passthrough";
  IplImage *cv_image_l = NULL, *cv_image_r = NULL, *cv_image_lt = NULL, *cv_image_rt = NULL;
  try
  {
    cv_image_l = bridge_l.imgMsgToCv(imagel, encoding.c_str());
    cv_image_r = bridge_r.imgMsgToCv(imager, encoding.c_str());
    cv_image_lt = bridge_lt.imgMsgToCv(imagelt, encoding.c_str());
    cv_image_rt = bridge_rt.imgMsgToCv(imagert, encoding.c_str());
  }
  catch (sensor_msgs::CvBridgeException error)
  {
    ROS_ERROR("BaseGenerator::visualFeaturesExtendedCallback: Could not convert image message to IplImage");
    return;
  }

  string path, left, right, leftt, rightt;

  left = "/left";
  right = "/right";
  leftt = "/left_tex";
  rightt = "/right_tex";

  path = directory + left;
  writeImage(path, cv_image_l);

  path = directory + right;
  writeImage(path, cv_image_r);

  path = directory + leftt;
  writeImage(path, cv_image_lt);

  path = directory + rightt;
  writeImage(path, cv_image_rt);

  file_count++;
}

void ImagesGenerator::visualFeaturesCallback(const ImageConstPtr &imagel, const ImageConstPtr &imager)
{
  ROS_INFO("visualFeaturesCallback");

  ROS_ASSERT(imagel);
  ROS_ASSERT(imager);

  string encoding = "passthrough";
  IplImage *cv_image_l = NULL, *cv_image_r = NULL;
  try
  {
    cv_image_l = bridge_l.imgMsgToCv(imagel, encoding.c_str());
    cv_image_r = bridge_r.imgMsgToCv(imager, encoding.c_str());
  }
  catch (sensor_msgs::CvBridgeException error)
  {
    ROS_ERROR("BaseGenerator::visualFeaturesCallback: Could not convert image message to IplImage");
    return;
  }
  string path, left, right;

  left = "/left";
  right = "/right";

  path = directory + left;
  writeImage(path, cv_image_l);

  path = directory + right;
  writeImage(path, cv_image_r);

  file_count++;
}

// a version for prosilica together with textured stereo
void ImagesGenerator::visualFeaturesProCallback(const ImageConstPtr &imagel, const ImageConstPtr &imager,
                                                const ImageConstPtr& imagepro)
{
  ROS_INFO("visualFeaturesProCallback");

  ROS_ASSERT(imagel);
  ROS_ASSERT(imager);
  ROS_ASSERT(imagepro);

  string encoding = "passthrough";
  IplImage *cv_image_l = NULL, *cv_image_r = NULL, *cv_image_pro = NULL;
  try
  {
    cv_image_l = bridge_l.imgMsgToCv(imagel, encoding.c_str());
    cv_image_r = bridge_r.imgMsgToCv(imager, encoding.c_str());
    if (imagepro->encoding.find("bayer") != std::string::npos)
      boost::const_pointer_cast<sensor_msgs::Image>(imagepro)->encoding = "mono8";
    cv_image_pro = bridge_pro.imgMsgToCv(imagepro, "bgr8");
  }
  catch (sensor_msgs::CvBridgeException error)
  {
    ROS_ERROR("BaseGenerator::visualFeaturesExtendedProCallback: Could not convert image message to IplImage");
    return;
  }

  string path, left, right, pro;

  left = "/left";
  right = "/right";
  pro = "/pro";

  path = directory + left;
  writeImage(path, cv_image_l);

  path = directory + right;
  writeImage(path, cv_image_r);

  path = directory + pro;
  writeImage(path, cv_image_pro);

  file_count++;
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_subscriber");
  ros::NodeHandle nh;
  ImagesGenerator generator(nh);
  generator.init();
  ros::spin();
  return 0;
}
