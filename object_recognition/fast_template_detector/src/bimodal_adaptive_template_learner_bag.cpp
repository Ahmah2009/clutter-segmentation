#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>
#include <stereo_msgs/DisparityImage.h>
#include <cv_bridge/CvBridge.h>
#include <image_transport/image_transport.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <ftd/fast_template_detector_vs.h>
#include <ftd/discretizers/gradient_discretizer.h>
#include <ftd/discretizers/surface_normal_discretizer.h>
#include <ftd/learners/bimodal_adaptive_template_learner.h>
#include <dot/DiscretizedData.h>
#include <dot/DetectionCandidates.h>
#include <io/mouse.h>
#include <util/timer.h>

#include "pcl/compute_normals.h"
#include <pcl/io/pcd_io.h>

#include <rosrecord/Player.h>

#include <sstream>
#include <string>


::ftd::learners::BimodalAdaptiveTemplateLearnerBAG * templateLearner;



int
main(
  int argc,
  char ** argv )
{
  using ::ftd::learners::BimodalAdaptiveTemplateLearnerBAG;

  ::ros::init (argc, argv, "bimodal_adaptive_template_learner_bag");
  ::ros::NodeHandle nodeHandle ("~");
  
  std::string bagFileName ((argc > 1) ? argv[1] : "all.bag");
  std::string templateFileName1 ((argc > 2) ? argv[2] : "test1.dot");
  std::string templateFileName2 ((argc > 3) ? argv[3] : "test2.dot");
  std::string imageTopic ("/narrow_stereo/left/image_raw");
  std::string camInfoTopic ("/narrow_stereo/left/camera_info");
  std::string discretizedDataTopic ("/discretized_data");
  //std::string cloudTopic ("/narrow_stereo_textured/points2");
  std::string cloudTopic ("/cloud_rectified");
  std::string disparityTopic ("/narrow_stereo_textured/disparity");
  
  const int detectionThreshold = (argc > 4) ? atoi (argv[4]) : 80;
  const int learningThreshold = (argc > 5) ? atoi (argv[5]) : 95;
  
  templateLearner = new ::ftd::learners::BimodalAdaptiveTemplateLearnerBAG (bagFileName, imageTopic, camInfoTopic, discretizedDataTopic, cloudTopic, disparityTopic, templateFileName1, templateFileName2, 154, 154, 7, 1, detectionThreshold, learningThreshold);
  
  templateLearner->spin ();
  
  delete templateLearner;

}

