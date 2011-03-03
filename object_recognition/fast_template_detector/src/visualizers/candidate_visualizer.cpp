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
#include <io/mouse.h>
#include <util/timer.h>

//#include "pcl/compute_normals.h"
#include <pcl/io/pcd_io.h>

#include <sstream>
#include <string>

//typedef ::message_filters::sync_policies::ApproximateTime< ::sensor_msgs::Image, ::sensor_msgs::Image > SyncPolicy;
typedef ::message_filters::sync_policies::ApproximateTime< ::sensor_msgs::Image, ::fast_template_detector::DetectionCandidates > SyncPolicy;


class CandidateVisualizer
{

public:
  
  CandidateVisualizer (
    ros::NodeHandle & nh,
    const int threshold,
    const std::string& transport,
    ::message_filters::Synchronizer<SyncPolicy> & sync_ )
  //: sync_ (SyncPolicy(20))
  {    
    threshold_ = threshold;
    
    cvNamedWindow ( "Image Window", CV_WINDOW_AUTOSIZE );
    
    sync_.registerCallback (boost::bind (&CandidateVisualizer::visualizationCallback, this, _1, _2));    
    
    ROS_INFO("system ready");
  }
  
  ~CandidateVisualizer ()
  {
  }
  

  void 
    visualizationCallback(
      const ::sensor_msgs::ImageConstPtr & msg,
//      const ::sensor_msgs::ImageConstPtr & msg2 )
      const ::fast_template_detector::DetectionCandidatesConstPtr & dataMsg )
  {
    ROS_INFO("Received data"); 
    
    
    // get OpenCV image
    IplImage * image = bridge_.imgMsgToCv(msg); 
  
  
    // create color image
    IplImage * colorImage = NULL;
    
    if (image->depth == 32)
    {
      if (image->nChannels == 3)
      {
        colorImage = static_cast<IplImage*>(cvClone(image));
      }
      else if (image->nChannels == 1)
      {
        colorImage = cvCreateImage(cvGetSize(image), IPL_DEPTH_32F, 3);
        cvCvtColor(image, colorImage, CV_GRAY2BGR);
      }
    }
    else if (image->depth == 8)
    {
      if (image->nChannels == 3)
      {
        colorImage = cvCreateImage(cvGetSize(image), IPL_DEPTH_32F, 3);
        cvConvert(image, colorImage);
      }
      else if (image->nChannels == 1)
      {
        IplImage * tmpGrayImage = cvCreateImage(cvGetSize(image), IPL_DEPTH_32F, 1);
        cvConvert(image, tmpGrayImage);
        
        colorImage = cvCreateImage(cvGetSize(image), IPL_DEPTH_32F, 3);
        cvCvtColor(tmpGrayImage, colorImage, CV_GRAY2BGR);
        
        cvReleaseImage(&tmpGrayImage);
      }
    }    
    
    
    // resize image
    const int imageWidth = 640;
    const int imageHeight = 480; 
    
    IplImage * resizedImage = cvCreateImage (cvSize (imageWidth, imageHeight), IPL_DEPTH_32F, 3);
    cvResize (colorImage, resizedImage);
        
    
    for (unsigned int candidateIndex = 0; candidateIndex < dataMsg->candidates.size (); ++candidateIndex)
    {
      const int templateCenterX = dataMsg->candidates[candidateIndex].center.x;
      const int templateCenterY = dataMsg->candidates[candidateIndex].center.y;
      const int templateWidth = dataMsg->candidates[candidateIndex].width;
      const int templateHeight = dataMsg->candidates[candidateIndex].height;
      
      const int templateStartX = templateCenterX - templateWidth/2;
      const int templateStartY = templateCenterY - templateHeight/2;
      const int templateEndX = templateStartX + templateWidth;
      const int templateEndY = templateStartY + templateHeight;
      
      cvLine (
        resizedImage,
        cvPoint (templateStartX, templateStartY),
        cvPoint (templateEndX, templateStartY),
        CV_RGB (0, 255, 0),
        2, 8, 0 );
      cvLine (
        resizedImage,
        cvPoint (templateEndX, templateStartY),
        cvPoint (templateEndX, templateEndY),
        CV_RGB (0, 255, 0),
        2, 8, 0 );
      cvLine (
        resizedImage,
        cvPoint (templateEndX, templateEndY),
        cvPoint (templateStartX, templateEndY),
        CV_RGB (0, 255, 0),
        2, 8, 0 );
      cvLine (
        resizedImage,
        cvPoint (templateStartX, templateEndY),
        cvPoint (templateStartX, templateStartY),
        CV_RGB (0, 255, 0),
        2, 8, 0 );
    }

 
    // display image
    cvScale (resizedImage, resizedImage, 1.0/255.0);
    cvShowImage ("Image Window", resizedImage);
    int pressedKey = cvWaitKey (10);


    cvReleaseImage (&colorImage);
    cvReleaseImage (&resizedImage);
  }
  
  
private:

  int threshold_;
  
  ::ros::Subscriber dataSubscriber_;
  ::sensor_msgs::CvBridge bridge_;
    
  //::message_filters::Synchronizer<SyncPolicy> sync_;
  
};



int
main(
  int argc,
  char ** argv )
{
  ::ros::init (argc, argv, "candidate_visualizer");
  ::ros::NodeHandle nodeHandle ("~");
  
/*    typedef ::message_filters::sync_policies::ApproximateTime< ::sensor_msgs::Image, ::sensor_msgs::Image > MySyncPolicy2;
        
    ::message_filters::Subscriber< ::sensor_msgs::Image > image_sub(nodeHandle, "/image", 1);
//    ::message_filters::Subscriber< ::fast_template_detector::DiscretizedData > data_sub(nodeHandle, "/discretized_data", 1);
    ::message_filters::Subscriber< ::sensor_msgs::Image > data_sub(nodeHandle, "/discretized_data", 1);
    
    ::message_filters::Synchronizer<MySyncPolicy2> sync (MySyncPolicy2(10), image_sub, data_sub);
    sync.registerCallback(boost::bind(&TemplateLearner::learnCallback, _1, _2));*/
    
    ::message_filters::Subscriber< ::sensor_msgs::Image > image_sub(nodeHandle, "/image", 20);
    ::message_filters::Subscriber< ::fast_template_detector::DetectionCandidates > data_sub(nodeHandle, "/detection_candidates", 20);
//    ::message_filters::Subscriber< ::sensor_msgs::Image > data_sub(nodeHandle, "/narrow_stereo/right/image_rect", 20);
    
    ::message_filters::Synchronizer<SyncPolicy> sync (SyncPolicy(20), image_sub, data_sub);
    sync.connectInput (image_sub, data_sub);
  
  CandidateVisualizer visualizer (nodeHandle, (argc > 1) ? atoi (argv[1]) : 80, (argc > 2) ? argv[2] : "raw", sync);
  
  ::ros::spin ();
}




