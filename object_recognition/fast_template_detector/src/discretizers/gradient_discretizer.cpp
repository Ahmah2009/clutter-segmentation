#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
//#include <nav_msgs/Odometry.h>
//#include <stereo_msgs/DisparityImage.h>
#include <cv_bridge/CvBridge.h>
#include <image_transport/image_transport.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <fast_template_detector/DiscretizedData.h>
//#include <io/mouse.h>
//#include <util/timer.h>
#include <util/image_utils.h>

#include <ftd/discretizers/gradient_discretizer.h>


#include <sstream>
#include <string>


class GradientDiscretizer
{

public:
  
  GradientDiscretizer (
    ros::NodeHandle & nh,
    const int numOfGradientsPerBin, 
    const std::string& transport )
  : numOfGradientsPerBin_ (numOfGradientsPerBin)
  {
    std::string topic = nh.resolveName("image");

    ::image_transport::ImageTransport it(nh);
    imageSubscriber_ = it.subscribe(topic, 1, &GradientDiscretizer::discretize, this, transport);
    
    discretizationPublisher_ = nh.advertise<fast_template_detector::DiscretizedData>("discretized_data", 100);
  }
  
  void 
    discretize(
      const ::sensor_msgs::ImageConstPtr & msg )
  {
    ROS_INFO("Received image"); 
    

    // get OpenCV image
    IplImage * image = bridge_.imgMsgToCv(msg); 
  
  
    // convert to gray value image
    IplImage * grayImage = util::ImageUtils::createGrayImage32F (image);
    
    
    // resize image
    const int imageWidth = 640;
    const int imageHeight = 480; 
    
    IplImage * resizedImage = cvCreateImage (cvSize (imageWidth, imageHeight), IPL_DEPTH_32F, 1);
    cvResize (grayImage, resizedImage);
    
    
    // smooth image
    IplImage * smoothedImage = cvCreateImage(cvSize (imageWidth, imageHeight), IPL_DEPTH_32F, 1);
    cvSmooth(resizedImage, smoothedImage, CV_GAUSSIAN, 5);
      
    
    const int regionWidth = 7;
    const int regionHeight = 7;
    
    const int horizontalSamples = imageWidth/regionWidth;
    const int verticalSamples = imageHeight/regionHeight;
    
    std::vector<unsigned char> data (horizontalSamples*verticalSamples, 0);

    
    // compute gradients
    ::ftd::discretizers::GradientDiscretizer::discretize (
      smoothedImage,
      regionWidth,
      regionHeight,
      data,
      numOfGradientsPerBin_ );
    
    
    // publish discretized data
    ::fast_template_detector::DiscretizedData dataMsg;
    dataMsg.header = msg->header;
    dataMsg.dataWidth = imageWidth;
    dataMsg.dataHeight = imageHeight;
    dataMsg.binWidth = regionWidth;
    dataMsg.binHeight = regionHeight;
    dataMsg.horizontalSamples = horizontalSamples;
    dataMsg.verticalSamples = verticalSamples;
    dataMsg.charsPerSample = 1;
    dataMsg.data = data;
    //dataMsg.strength = strength;
    
    discretizationPublisher_.publish(dataMsg);
    
    ROS_INFO("published message");
    
    
    // display image
    cvScale (resizedImage, resizedImage, 1.0/255.0);
    cvShowImage ("Image Window", resizedImage);
    cvWaitKey (10);

    
    // release data
    cvReleaseImage (&grayImage);
    cvReleaseImage (&resizedImage);
    cvReleaseImage (&smoothedImage);
  }
  
  
private:

  /** \brief num of gradients used per bin */
  int numOfGradientsPerBin_;

  ::image_transport::Subscriber imageSubscriber_;
  ::ros::Publisher discretizationPublisher_;
  ::sensor_msgs::CvBridge bridge_;
  
};



int
main(
  int argc,
  char ** argv )
{
  cvNamedWindow ( "Image Window", CV_WINDOW_AUTOSIZE );

  ::ros::init (argc, argv, "gradient_discretizer");
  ::ros::NodeHandle nodeHandle;
  
  const int numOfGradientsPerBin = (argc > 1) ? atoi (argv[1]) : 8;
  
  GradientDiscretizer discretizer (nodeHandle, numOfGradientsPerBin, (argc > 2) ? argv[2] : "raw");
  
  ::ros::spin ();
}


