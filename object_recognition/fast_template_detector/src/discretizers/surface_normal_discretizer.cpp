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

#include <ftd/discretizers/surface_normal_discretizer.h>

#include <fast_template_detector/DiscretizedData.h>
#include <io/mouse.h>
#include <util/timer.h>
#include <util/image_utils.h>

#include "pcl/compute_normals.h"
#include <pcl/io/pcd_io.h>

#include <sstream>
#include <string>

//typedef ::message_filters::sync_policies::ApproximateTime< ::sensor_msgs::PointCloud2, ::stereo_msgs::DisparityImage > SyncPolicy;
typedef ::message_filters::sync_policies::ApproximateTime< ::sensor_msgs::Image, ::sensor_msgs::PointCloud2 > SyncPolicy;

class SurfaceNormalDiscretizer
{

public:
  
  SurfaceNormalDiscretizer (
    ros::NodeHandle & nh, 
    const std::string& transport,
    ::message_filters::Synchronizer<SyncPolicy> & sync_ )
  {
//    std::string topic = nh.resolveName("points2");

//    cloudSubscriber_ = nh.subscribe< sensor_msgs::PointCloud2 > (topic, 1, &SurfaceNormalDiscretizer::discretize, this);
    discretizationPublisher_ = nh.advertise< ::fast_template_detector::DiscretizedData > ("discretized_data", 100);

    sync_.registerCallback (boost::bind (&SurfaceNormalDiscretizer::discretize, this, _1, _2));    
  }
  
//  void 
//    discretize(
//      const ::sensor_msgs::PointCloud2ConstPtr & cloud2,
//      const ::stereo_msgs::DisparityImageConstPtr & disparityMsg )
  void 
    discretize(
      const ::sensor_msgs::ImageConstPtr & imageMsg,
      const ::sensor_msgs::PointCloud2ConstPtr & cloud2 )
  {
    ROS_INFO("Received image"); 
    
    
    // get OpenCV image
    IplImage * image = bridge_.imgMsgToCv(imageMsg); 
  
  
    // convert to gray value image
    IplImage * grayImage = util::ImageUtils::createGrayImage32F (image);

   
    // discretize data
    const int imageWidth = grayImage->width;
    const int imageHeight = grayImage->height;
    
    const int regionWidth = 7;
    const int regionHeight = 7;
    
    const int horizontalSamples = imageWidth/regionWidth;
    const int verticalSamples = imageHeight/regionHeight;
    
    std::vector<unsigned char> data(horizontalSamples*verticalSamples);
    
    ::ftd::discretizers::SurfaceNormalDiscretizer::discretize (
      cloud2,
      regionWidth,
      regionHeight,
      data );    


    // publish discretized data
    ::fast_template_detector::DiscretizedData dataMsg;
    dataMsg.header = cloud2->header;
    dataMsg.dataWidth = imageWidth;
    dataMsg.dataHeight = imageHeight;
    dataMsg.binWidth = regionWidth;
    dataMsg.binHeight = regionHeight;
    dataMsg.horizontalSamples = horizontalSamples;
    dataMsg.verticalSamples = verticalSamples;
    dataMsg.charsPerSample = 1;
    dataMsg.data = data;
    
    discretizationPublisher_.publish(dataMsg);
    
    ROS_INFO("published message");

    
    cvScale (grayImage, grayImage, 1.0/255.0);
    cvShowImage ("Image Window", grayImage);
    cvWaitKey (10);
    
    
    cvReleaseImage (&grayImage);
  }
  
  
private:

  ::ros::Subscriber cloudSubscriber_;
  ::ros::Publisher discretizationPublisher_;
  ::sensor_msgs::CvBridge bridge_;
  
  ComputeNormals cn_;
  
};



int
main(
  int argc,
  char ** argv )
{
  cvNamedWindow ( "Image Window", CV_WINDOW_AUTOSIZE );

  ::ros::init (argc, argv, "surface_normal_discretizer");
  ::ros::NodeHandle nodeHandle;

  ::message_filters::Subscriber< ::sensor_msgs::Image > image_sub(nodeHandle, "/image", 20);
  ::message_filters::Subscriber< ::sensor_msgs::PointCloud2 > pointCloud2_sub(nodeHandle, "/points2", 100);
//  ::message_filters::Subscriber< ::stereo_msgs::DisparityImage > disparity_sub(nodeHandle, "/disparity", 100);
  
//  ::message_filters::Synchronizer<SyncPolicy> sync (SyncPolicy(100), pointCloud2_sub, disparity_sub);
//  sync.connectInput (pointCloud2_sub, disparity_sub);
  ::message_filters::Synchronizer<SyncPolicy> sync (SyncPolicy(100), image_sub, pointCloud2_sub);
  sync.connectInput (image_sub, pointCloud2_sub);
  
  SurfaceNormalDiscretizer discretizer (nodeHandle, (argc > 1) ? argv[1] : "raw", sync);
  
  ::ros::spin ();
}


