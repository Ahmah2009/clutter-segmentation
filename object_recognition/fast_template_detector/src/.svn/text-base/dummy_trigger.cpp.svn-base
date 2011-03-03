#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <stereo_msgs/DisparityImage.h>
#include <cv_bridge/CvBridge.h>
#include <image_transport/image_transport.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <ftd/fast_template_detector_vs.h>
#include <fast_template_detector/DiscretizedData.h>
#include <fast_template_detector/DetectionCandidates.h>
#include <fast_template_detector/Trigger.h>
#include <io/mouse.h>
#include <util/timer.h>

//#include "pcl/compute_normals.h"
#include <pcl/io/pcd_io.h>

#include "topic_tools/shape_shifter.h"
#include "std_msgs/Empty.h"

#include <sstream>
#include <string>


class DummyTrigger
{

public:
  
  DummyTrigger (
    ros::NodeHandle & nh,
    int argc,
    char ** argv )
  {
    int g_count = -1;
    
    // Every non-processed argument is assumed to be a topic
    for (int i = 1; i < argc; i++)
    {
      boost::shared_ptr<int> count(new int(g_count));
      boost::shared_ptr<ros::Subscriber> sub(new ros::Subscriber);
//      g_num_subscribers++;
      *sub = nh.subscribe<topic_tools::ShapeShifter>(
        argv[i], 
        100, 
        boost::bind(&DummyTrigger::callback, this, _1, argv[i], sub, count) );
    }
   
    publisher_ = nh.advertise<fast_template_detector::Trigger>("/trigger", 100);
   
//    ros::MultiThreadedSpinner s(10);
//    ros::spin(s);
    ros::spin();
  
    
  
//    subscriber_ = nh.subscribe<fast_template_detector::DiscretizedData>(topic, 1, &DummyTrigger::callback, this);
//    
  }
  
  ~DummyTrigger ()
  {
  }
  
  void 
    callback(
      topic_tools::ShapeShifter::ConstPtr msg,
      std::string topic_name,
      boost::shared_ptr<ros::Subscriber> subscriber,
      boost::shared_ptr<int> count )
  {
    ROS_INFO("Received data"); 

    ::fast_template_detector::Trigger triggerMsg;
    
    publisher_.publish(triggerMsg);
  }
  
  
private:

  ::ros::Subscriber subscriber_;
  ::ros::Publisher publisher_;
  
};



int
main(
  int argc,
  char ** argv )
{
  ::ros::init (argc, argv, "dummy_trigger");
  ::ros::NodeHandle nodeHandle;
  
  DummyTrigger dummyTrigger (nodeHandle, argc, argv);
  
  ::ros::spin ();
}



