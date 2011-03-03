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


class MultiClassCandidateDetector
{

public:
  
  MultiClassCandidateDetector (
    ros::NodeHandle & nh,
    const std::string& templateFileName, 
    const int threshold,
    const std::string& transport )
  {
    std::string topic = nh.resolveName("discretized_data");
  
    dataSubscriber_ = nh.subscribe<fast_template_detector::DiscretizedData>(topic, 1, &MultiClassCandidateDetector::detect, this);
    candidatePublisher_ = nh.advertise<fast_template_detector::DetectionCandidates>("detection_candidates", 100);
    
    threshold_ = threshold;
    
    const int regionSize = 7;
    const int templateHorizontalSamples = 154/regionSize;
    const int templateVerticalSamples = 154/regionSize;
    const int numOfCharsPerElement = 1;
    
    templateDetector_ = new ::ftd::FastTemplateDetectorVS(templateHorizontalSamples, templateVerticalSamples, regionSize, numOfCharsPerElement, 10);
    
    templateDetector_->load(templateFileName);
    templateDetector_->clearClusters ();
    templateDetector_->clusterHeuristically (4);
  }
  
  ~MultiClassCandidateDetector ()
  {
    delete templateDetector_;
  }
  
  void 
    detect(
      const ::fast_template_detector::DiscretizedDataConstPtr & dataMsg )
  {
    ROS_INFO("Received data"); 
    
    std::cerr << dataMsg->header.stamp << std::endl;
    
    
    // copy data
    const int horizontalSamples = dataMsg->horizontalSamples;
    const int verticalSamples = dataMsg->verticalSamples;
    const int regionWidth = dataMsg->binWidth;
    const int regionHeight = dataMsg->binHeight;
    
    //unsigned char * data = new unsigned char[horizontalSamples*verticalSamples];
    unsigned char * data = reinterpret_cast<unsigned char*>(_mm_malloc(sizeof(unsigned char)*horizontalSamples*verticalSamples, 16));
    
    int elementIndex = 0;
    for ( std::vector<unsigned char>::const_iterator iter = dataMsg->data.begin (); 
          iter != dataMsg->data.end (); 
          ++iter)
    {
      data[elementIndex] = *iter;
      
      ++elementIndex;
    }
    
    
    std::cerr << "numOfClasses: " << templateDetector_->getNumOfClasses() << std::endl;
    std::cerr << "numOfTemplates: " << templateDetector_->getNumOfTemplates() << std::endl;
    

    // detect templates
    const int threshold = threshold_;
    
    
    ::ros::Time t1 = ::ros::Time::now ();
    const bool detectOnlyBestCandidatesPerPosition = true;
    std::list< ::ftd::Candidate* > * candidateList = templateDetector_->process(data, threshold, dataMsg->horizontalSamples, dataMsg->verticalSamples, detectOnlyBestCandidatesPerPosition);
    ::ros::Time t2 = ::ros::Time::now ();
    
    ROS_INFO ("detection time: %f", (t2-t1).toSec ());

    ::fast_template_detector::DetectionCandidates detectionCandidates;
    detectionCandidates.header.stamp = dataMsg->header.stamp;

    if (candidateList != NULL)
    {
    
		  for (int classIndex = 0; classIndex < templateDetector_->getNumOfClasses(); ++classIndex)
		  {			
			  for (std::list< ::ftd::Candidate* >::iterator candidateIter = candidateList[classIndex].begin(); candidateIter != candidateList[classIndex].end(); ++candidateIter)
			  {
					  const int regionStartX = (*candidateIter)->getCol ();
					  const int regionStartY = (*candidateIter)->getRow ();
					  
					  //std::cerr << "---------------" << std::endl;
					  //std::cerr << "response: " << (*candidateIter)->getMatchingResponse () << std::endl;
					  //std::cerr << "classId: " << classIndex << std::endl;
					  //std::cerr << "templateId: " << (*candidateIter)->getIndex ()-1 << std::endl;
            
            ::fast_template_detector::DetectionCandidate candidate;
            candidate.response = (*candidateIter)->getMatchingResponse ();
            candidate.classId = classIndex;
            candidate.templateId = (*candidateIter)->getIndex ()-1;
            candidate.center.x = regionStartX+templateDetector_->getTemplateWidth ()/2;
            candidate.center.y = regionStartY+templateDetector_->getTemplateHeight ()/2;
            candidate.width = templateDetector_->getTemplateWidth ();
            candidate.height = templateDetector_->getTemplateHeight ();
            
            detectionCandidates.candidates.push_back (candidate);
			  }
			}
			
		  for (int classIndex = 0; classIndex < templateDetector_->getNumOfClasses(); ++classIndex)
		  {
			  ::ftd::emptyPointerList(candidateList[classIndex]);
		  }

      delete[] candidateList;
		}

  	candidatePublisher_.publish(detectionCandidates);
  	
  	//::ros::spinOnce ();
    

    //delete[] data;
    _mm_free(data);
  }
  
  
private:

  int threshold_;

  ::ros::Subscriber dataSubscriber_;
  ::ros::Publisher candidatePublisher_;
  ::sensor_msgs::CvBridge bridge_;
  
  ::ftd::FastTemplateDetectorVS * templateDetector_;
  
};



int
main(
  int argc,
  char ** argv )
{
  cvNamedWindow ( "Image Window", CV_WINDOW_AUTOSIZE );

  ::ros::init (argc, argv, "multi_class_candidate_detector");
  ::ros::NodeHandle nodeHandle;
  
  MultiClassCandidateDetector detector (nodeHandle, (argc > 1) ? argv[1] : "test.dot", (argc > 2) ? atoi (argv[2]) : 100, (argc > 3) ? argv[3] : "raw");
  
  ::ros::spin ();
}



