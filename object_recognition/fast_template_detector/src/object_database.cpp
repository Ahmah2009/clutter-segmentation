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
#include <fast_template_detector/ObjectQuery.h>
#include <fast_template_detector/ObjectQueryResponse.h>
#include <fast_template_detector/TemplateQuery.h>
#include <fast_template_detector/TemplateQueryResponse.h>
#include <io/mouse.h>
#include <util/timer.h>

#include <pcl/io/pcd_io.h>

#include <sstream>
#include <string>


class ObjectDatabase
{

public:
  
  ObjectDatabase (
    ros::NodeHandle & nh,
    const std::string& templateFileName, 
    const std::string& transport )
  {
    std::string topic1 = nh.resolveName("template_query");
    std::string topic2 = nh.resolveName("object_query");
  
    templateQuerySubscriber_ = nh.subscribe<fast_template_detector::TemplateQuery>(topic1, 1, &ObjectDatabase::templateQuery, this);
    objectQuerySubscriber_ = nh.subscribe<fast_template_detector::ObjectQuery>(topic2, 1, &ObjectDatabase::objectQuery, this);
    templateQueryResponsePublisher_ = nh.advertise<fast_template_detector::TemplateQueryResponse>("template_query_response", 100);
    objectQueryResponsePublisher_ = nh.advertise<fast_template_detector::ObjectQueryResponse>("object_query_response", 100);
    
    const int regionSize = 7;
    const int templateHorizontalSamples = 154/regionSize;
    const int templateVerticalSamples = 154/regionSize;
    const int numOfCharsPerElement = 1;
    
    templateDetector_ = new ::ftd::FastTemplateDetectorVS(templateHorizontalSamples, templateVerticalSamples, regionSize, numOfCharsPerElement, 10);
    
    templateDetector_->load(templateFileName);
    templateDetector_->clearClusters ();
    templateDetector_->clusterHeuristically (4);
  }
  
  ~ObjectDatabase ()
  {
    delete templateDetector_;
  }
  
  void 
    templateQuery(
      const ::fast_template_detector::TemplateQueryConstPtr & queryMsg )
  {
    ROS_INFO("Received data"); 
    
    const int classId = queryMsg->objectId;
    
    const int samplingSize = templateDetector_->getSamplingSize();
    const int templateWidth = templateDetector_->getTemplateWidth();
    const int templateHeight = templateDetector_->getTemplateHeight();
    
    const int bytesPerTemplate = (templateWidth/samplingSize) * (templateHeight/samplingSize);
    
    int templateCounter = 0;
    std::vector< ::fast_template_detector::Template > bitlists;
    ::ftd::List * currentTemplateData = templateDetector_->getTemplateList();
    while (currentTemplateData != NULL)
    {
      if (currentTemplateData->getClassIndex() == classId)
      {
        unsigned char * bitlist = currentTemplateData->getBitList();
        std::vector<unsigned char> bitListVector;
        for (int i = 0; i < bytesPerTemplate; ++i)
        {
          bitListVector.push_back(bitlist[i]);
        }
        
        ::fast_template_detector::Template templateData;
        templateData.data = bitListVector;
        
        bitlists.push_back(templateData);
    
        ++templateCounter;
      }
      
      currentTemplateData = currentTemplateData->getNext();
    }
    
    
    ::fast_template_detector::TemplateQueryResponse responseMsg;
    
    responseMsg.numOfTemplates = bitlists.size();
    responseMsg.bytesPerTemplate = bytesPerTemplate;
    responseMsg.templates = bitlists;
    
    templateQueryResponsePublisher_.publish(responseMsg);
  }
  
  void 
    objectQuery(
      const ::fast_template_detector::ObjectQueryConstPtr & queryMsg )
  {
    ROS_INFO("Received data"); 
    
    ::fast_template_detector::ObjectQueryResponse responseMsg;
    
    responseMsg.numOfObjectsInDatabase = templateDetector_->getNumOfClasses ();
    
    objectQueryResponsePublisher_.publish(responseMsg);
  }
  
  
private:

  ::ros::Subscriber templateQuerySubscriber_;
  ::ros::Subscriber objectQuerySubscriber_;
  ::ros::Publisher templateQueryResponsePublisher_;
  ::ros::Publisher objectQueryResponsePublisher_;
  
  ::ftd::FastTemplateDetectorVS * templateDetector_;
  
};



int
main(
  int argc,
  char ** argv )
{
  cvNamedWindow ( "Image Window", CV_WINDOW_AUTOSIZE );

  ::ros::init (argc, argv, "object_database");
  ::ros::NodeHandle nodeHandle;
  
  ObjectDatabase database (nodeHandle, (argc > 1) ? argv[1] : "test.ftd", (argc > 2) ? argv[2] : "raw");
  
  ::ros::spin ();
}



