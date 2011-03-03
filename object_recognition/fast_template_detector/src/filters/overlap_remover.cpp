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

#include <pcl/io/pcd_io.h>

#include <sstream>
#include <string>

//typedef ::message_filters::sync_policies::ApproximateTime< ::sensor_msgs::Image, ::sensor_msgs::Image > SyncPolicy;
typedef ::message_filters::sync_policies::ApproximateTime< ::sensor_msgs::Image, ::fast_template_detector::DetectionCandidates > SyncPolicy;


CvScalar colors[] = {
CV_RGB(0, 0, 0),
CV_RGB(255, 0, 0),
CV_RGB(0, 255, 0),
CV_RGB(0, 0, 255),
CV_RGB(255, 255, 0),
CV_RGB(255, 0, 255),
CV_RGB(0, 255, 255),
CV_RGB(255, 255, 255),
CV_RGB(128, 0, 0),
CV_RGB(0, 128, 0),
CV_RGB(0, 0, 128),
CV_RGB(128, 128, 0),
CV_RGB(128, 0, 128),
CV_RGB(0, 128, 128),
CV_RGB(128, 128, 128),
CV_RGB(255, 128, 128),
CV_RGB(128, 255, 128),
CV_RGB(128, 128, 255),
CV_RGB(255, 255, 128),
CV_RGB(255, 128, 255),
CV_RGB(128, 255, 255),
CV_RGB(255, 255, 255)
};


class OverlapRemover
{

public:
  
  OverlapRemover (
    ros::NodeHandle & nh,
    const float threshold,
    const std::string& transport,
    ::message_filters::Synchronizer<SyncPolicy> & sync_ )
  //: sync_ (SyncPolicy(20))
  {    
    threshold_ = threshold;
    
    std::cerr << threshold << std::endl;
    
    cvNamedWindow ( "Image Window", CV_WINDOW_AUTOSIZE );
    
    sync_.registerCallback (boost::bind (&OverlapRemover::filter, this, _1, _2));    
    
    candidatePublisher_ = nh.advertise<fast_template_detector::DetectionCandidates>("/detection_candidates_out", 100);

    
    ROS_INFO("system ready");
  }
  
  ~OverlapRemover ()
  {
  }
  

  void 
    filter(
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
    
    
    std::vector<int> detectedClasses;
    std::vector<CvRect> detectedTemplateRegions;
    std::vector<float> detectedClassResponses;
    for (unsigned int candidateIndex = 0; candidateIndex < dataMsg->candidates.size (); ++candidateIndex)
    {
      detectedClasses.push_back (candidateIndex);
      detectedTemplateRegions.push_back (cvRect (
        dataMsg->candidates[candidateIndex].center.x - dataMsg->candidates[candidateIndex].width/2,
        dataMsg->candidates[candidateIndex].center.y - dataMsg->candidates[candidateIndex].height/2,
        dataMsg->candidates[candidateIndex].width,
        dataMsg->candidates[candidateIndex].height ));
      detectedClassResponses.push_back (dataMsg->candidates[candidateIndex].response);
    }
        
    
    // compute overlap between template regions of detection classes
    std::set<int> finalClasses;
    for (unsigned int index1 = 0; index1 < detectedClasses.size (); ++index1)
    {
      finalClasses.insert (index1);
    }
    
    for (unsigned int index1 = 0; index1 < detectedClasses.size (); ++index1)
    {
      const float center1X = (detectedTemplateRegions[index1].x+detectedTemplateRegions[index1].width)/2.0f;
      const float center1Y = (detectedTemplateRegions[index1].y+detectedTemplateRegions[index1].height)/2.0f;
      
      for (unsigned int index2 = index1+1; index2 < detectedClasses.size (); ++index2)
      {
        const float center2X = (detectedTemplateRegions[index2].x+detectedTemplateRegions[index2].width)/2.0f;
        const float center2Y = (detectedTemplateRegions[index2].y+detectedTemplateRegions[index2].height)/2.0f;
        
        const float minWidth = std::min(detectedTemplateRegions[index1].width, detectedTemplateRegions[index2].width);
        const float minHeight = std::min(detectedTemplateRegions[index1].height, detectedTemplateRegions[index2].height);
        
        const float overlapWidth = minWidth - abs(center1X - center2X);
        const float overlapHeight = minHeight - abs(center1Y - center2Y);
        
        const float overlap = overlapWidth*overlapHeight/(minWidth*minHeight);
        
        if (overlap > threshold_)
        {
          if (detectedClassResponses[index1] > detectedClassResponses[index2])
          {
            finalClasses.erase (index2);
          }
          else
          {
            finalClasses.erase (index1);
          }
        }
        
        ROS_INFO ("OVERLAP: %f, %f", overlap, threshold_);
      }
    }
    
    ::fast_template_detector::DetectionCandidates detectionCandidates;
    detectionCandidates.header.stamp = dataMsg->header.stamp;
    
    for (std::set<int>::iterator iter = finalClasses.begin(); iter != finalClasses.end(); ++iter)
    {
      ROS_INFO("-------------------------------detected class: %d", detectedClasses[*iter]);
		  const int regionStartX = detectedTemplateRegions[*iter].x;
		  const int regionStartY = detectedTemplateRegions[*iter].y;
		  const int regionWidth = detectedTemplateRegions[*iter].width;
		  const int regionHeight = detectedTemplateRegions[*iter].height;
		  
      const int templateCenterX = dataMsg->candidates[*iter].center.x;
      const int templateCenterY = dataMsg->candidates[*iter].center.y;
      const int templateWidth = dataMsg->candidates[*iter].width;
      const int templateHeight = dataMsg->candidates[*iter].height;
      
      const int templateStartX = templateCenterX - templateWidth/2;
      const int templateStartY = templateCenterY - templateHeight/2;
      const int templateEndX = templateStartX + templateWidth;
      const int templateEndY = templateStartY + templateHeight;		  

      const int classId = dataMsg->candidates[*iter].classId;
      cvLine (
        resizedImage,
        cvPoint (templateStartX, templateStartY),
        cvPoint (templateEndX, templateStartY),
        colors[classId],
        2, 8, 0 );
      cvLine (
        resizedImage,
        cvPoint (templateEndX, templateStartY),
        cvPoint (templateEndX, templateEndY),
        colors[classId],
        2, 8, 0 );
      cvLine (
        resizedImage,
        cvPoint (templateEndX, templateEndY),
        cvPoint (templateStartX, templateEndY),
        colors[classId],
        2, 8, 0 );
      cvLine (
        resizedImage,
        cvPoint (templateStartX, templateEndY),
        cvPoint (templateStartX, templateStartY),
        colors[classId],
        2, 8, 0 );    
        
      std::stringstream ss;
      CvFont font;
      cvInitFont (&font, CV_FONT_HERSHEY_DUPLEX, 0.7, 0.7);
      ss << dataMsg->candidates[*iter].classId;
      cvPutText (
        resizedImage,
        ss.str ().c_str (),
        cvPoint (templateStartX, templateStartY),
        &font,
        CV_RGB (255, 0, 0) );
        
      ::fast_template_detector::DetectionCandidate candidate;
      candidate.response = dataMsg->candidates[*iter].response;
      candidate.classId = dataMsg->candidates[*iter].classId;
      candidate.templateId = dataMsg->candidates[*iter].templateId;
      candidate.center.x = dataMsg->candidates[*iter].center.x;
      candidate.center.y = dataMsg->candidates[*iter].center.y;
      candidate.width = dataMsg->candidates[*iter].width;
      candidate.height = dataMsg->candidates[*iter].height;
      
      detectionCandidates.candidates.push_back (candidate);
    }    
    
  	candidatePublisher_.publish(detectionCandidates);
  	
  	//::ros::spinOnce ();
  	    
        
/*    ::fast_template_detector::DetectionCandidates detectionCandidates;
    detectionCandidates.header.stamp = dataMsg->header.stamp;

    for (std::map<int, int>::iterator iter = classToBestCandidateMap.begin (); iter != classToBestCandidateMap.end (); ++iter)
    {
      const int templateCenterX = dataMsg->candidates[iter->second].center.x;
      const int templateCenterY = dataMsg->candidates[iter->second].center.y;
      const int templateWidth = dataMsg->candidates[iter->second].width;
      const int templateHeight = dataMsg->candidates[iter->second].height;
      
      const int templateStartX = templateCenterX - templateWidth/2;
      const int templateStartY = templateCenterY - templateHeight/2;
      const int templateEndX = templateStartX + templateWidth;
      const int templateEndY = templateStartY + templateHeight;
      
      ::fast_template_detector::DetectionCandidate candidate;
      candidate.response = dataMsg->candidates[iter->second].response;
      candidate.classId = dataMsg->candidates[iter->second].classId;
      candidate.templateId = dataMsg->candidates[iter->second].templateId;
      candidate.center.x = dataMsg->candidates[iter->second].center.x;
      candidate.center.y = dataMsg->candidates[iter->second].center.y;
      candidate.width = dataMsg->candidates[iter->second].width;
      candidate.height = dataMsg->candidates[iter->second].height;
      
      detectionCandidates.candidates.push_back (candidate);
      
      const int classId = dataMsg->candidates[iter->second].classId;
      cvLine (
        resizedImage,
        cvPoint (templateStartX, templateStartY),
        cvPoint (templateEndX, templateStartY),
        colors[classId],
        2, 8, 0 );
      cvLine (
        resizedImage,
        cvPoint (templateEndX, templateStartY),
        cvPoint (templateEndX, templateEndY),
        colors[classId],
        2, 8, 0 );
      cvLine (
        resizedImage,
        cvPoint (templateEndX, templateEndY),
        cvPoint (templateStartX, templateEndY),
        colors[classId],
        2, 8, 0 );
      cvLine (
        resizedImage,
        cvPoint (templateStartX, templateEndY),
        cvPoint (templateStartX, templateStartY),
        colors[classId],
        2, 8, 0 );    
        
      std::stringstream ss;
      CvFont font;
      cvInitFont (&font, CV_FONT_HERSHEY_DUPLEX, 0.7, 0.7);
      ss << dataMsg->candidates[iter->second].classId;
      cvPutText (
        resizedImage,
        ss.str ().c_str (),
        cvPoint (templateStartX, templateStartY),
        &font,
        CV_RGB (255, 0, 0) );
    }
    
  	candidatePublisher_.publish(detectionCandidates);
  	
  	::ros::spinOnce ();*/
    
  

 
    // display image
    cvScale (resizedImage, resizedImage, 1.0/255.0);
    cvShowImage ("Image Window", resizedImage);
    int pressedKey = cvWaitKey (10);


    cvReleaseImage (&colorImage);
    cvReleaseImage (&resizedImage);
  }
  
  
private:

  float threshold_;
  
  ::ros::Subscriber dataSubscriber_;
  ::ros::Publisher candidatePublisher_;
  ::sensor_msgs::CvBridge bridge_;
    
  //::message_filters::Synchronizer<SyncPolicy> sync_;
  
};



int
main(
  int argc,
  char ** argv )
{
  ::ros::init (argc, argv, "overlap_remover");
  ::ros::NodeHandle nodeHandle ("~");
  
/*    typedef ::message_filters::sync_policies::ApproximateTime< ::sensor_msgs::Image, ::sensor_msgs::Image > MySyncPolicy2;
        
    ::message_filters::Subscriber< ::sensor_msgs::Image > image_sub(nodeHandle, "/image", 1);
//    ::message_filters::Subscriber< ::fast_template_detector::DiscretizedData > data_sub(nodeHandle, "/discretized_data", 1);
    ::message_filters::Subscriber< ::sensor_msgs::Image > data_sub(nodeHandle, "/discretized_data", 1);
    
    ::message_filters::Synchronizer<MySyncPolicy2> sync (MySyncPolicy2(10), image_sub, data_sub);
    sync.registerCallback(boost::bind(&TemplateLearner::learnCallback, _1, _2));*/
    
    ::message_filters::Subscriber< ::sensor_msgs::Image > image_sub(nodeHandle, "/image", 20);
    ::message_filters::Subscriber< ::fast_template_detector::DetectionCandidates > data_sub(nodeHandle, "/detection_candidates_in", 20);
//    ::message_filters::Subscriber< ::sensor_msgs::Image > data_sub(nodeHandle, "/narrow_stereo/right/image_rect", 20);
    
    ::message_filters::Synchronizer<SyncPolicy> sync (SyncPolicy(20), image_sub, data_sub);
    sync.connectInput (image_sub, data_sub);
    
  const float overlapThreshold = static_cast<float>((argc > 1) ? atoi (argv[1]) : 90.0f)/100.0f;
  
  std::cerr << overlapThreshold << std::endl;
  
  OverlapRemover filter (nodeHandle, overlapThreshold, (argc > 2) ? argv[2] : "raw", sync);
  
  ::ros::spin ();
}




