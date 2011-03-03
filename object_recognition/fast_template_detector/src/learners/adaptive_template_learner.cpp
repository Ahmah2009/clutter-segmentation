#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
//#include <nav_msgs/Odometry.h>
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
#include <util/image_utils.h>

//#include "pcl/compute_normals.h"
//#include <pcl/io/pcd_io.h>

#include <sstream>
#include <string>

#include <ftd/discretizers/gradient_discretizer.h>

typedef ::message_filters::sync_policies::ApproximateTime< ::sensor_msgs::Image, ::fast_template_detector::DiscretizedData > SyncPolicy;


class AdaptiveTemplateLearner
{

public:
  
  AdaptiveTemplateLearner (
    ros::NodeHandle & nh,
    const std::string& templateFileName, 
    const int threshold,
    const int learnThreshold,
    const std::string& transport,
    ::message_filters::Synchronizer<SyncPolicy> & sync_ )
  {
    fileName_ = templateFileName;
    
    threshold_ = threshold;
    learnThreshold_ = learnThreshold;
    
    roi_ = cvRect(0, 0, 640, 480);
    
    const int regionSize = 7;
    const int templateHorizontalSamples = 154/regionSize;
    const int templateVerticalSamples = 154/regionSize;
//    const int templateHorizontalSamples = 56/regionSize;
//    const int templateVerticalSamples = 56/regionSize;
    const int numOfCharsPerElement = 1;
    
    templateDetector_ = new ::ftd::FastTemplateDetectorVS (templateHorizontalSamples, templateVerticalSamples, regionSize, numOfCharsPerElement, 10);
    templateDetector_->addNewClass ();
        
    cvNamedWindow ( "Image Window", CV_WINDOW_AUTOSIZE );
    mouse_.start("Image Window");
    
    sync_.registerCallback (boost::bind (&AdaptiveTemplateLearner::learnCallback, this, _1, _2));    
    
    ROS_INFO("system ready");
   
    counter_ = 0;
  }
  
  ~AdaptiveTemplateLearner ()
  {
    delete templateDetector_;
  }
  
  void 
    learnCallback(
      const ::sensor_msgs::ImageConstPtr & msg,
      const ::fast_template_detector::DiscretizedDataConstPtr & dataMsg )
  {
    ROS_INFO("Received data"); 
    
    
    // get OpenCV image
    IplImage * image = bridge_.imgMsgToCv(msg); 
  
  
    // create color image
    IplImage * colorImage = util::ImageUtils::createColorImage32F (image);
    IplImage * colorImage2 = static_cast<IplImage*>(cvClone(colorImage));
    
    
    // convert to gray value image
    IplImage * grayImage = util::ImageUtils::createGrayImage32F (image);
        
    const int imageWidth = grayImage->width;
    const int imageHeight = grayImage->height; 
    
//    IplImage * colorImage = cvCreateImage (cvSize (imageWidth, imageHeight), IPL_DEPTH_32F, 3);
//    cvResize (colorImage, colorImage);
    
    IplImage * smoothedImage = cvCreateImage (cvGetSize (grayImage), IPL_DEPTH_32F, 1);
    cvSmooth (grayImage, smoothedImage, CV_BLUR, 5);
    
    
    // get current mouse position
    const int mousePosX = mouse_.getX ();
    const int mousePosY = mouse_.getY ();
    const int mouseEvent = mouse_.getEvent();
    
    
    // visualize template area
    int templateCenterX = -1;
    int templateCenterY = -1;
    if (mousePosX >= 0 && mousePosY >= 0)
    {
	    templateCenterX = mousePosX;
	    templateCenterY = mousePosY;		
    }
    if ( templateCenterX - templateDetector_->getTemplateWidth()/2 >= 0 ||
	       templateCenterY - templateDetector_->getTemplateHeight()/2 >= 0 ||
	       templateCenterX + templateDetector_->getTemplateWidth()/2 <= colorImage->width-1 ||
	       templateCenterY + templateDetector_->getTemplateHeight()/2 <= colorImage->height-1 )
    {
	    CvPoint pt1;
	    CvPoint pt2;
	    CvPoint pt3;
	    CvPoint pt4;

	    pt1.x = templateCenterX-templateDetector_->getTemplateWidth()/2;
	    pt1.y = templateCenterY-templateDetector_->getTemplateHeight()/2;
	    pt2.x = templateCenterX+templateDetector_->getTemplateWidth()/2;
	    pt2.y = templateCenterY-templateDetector_->getTemplateHeight()/2;
	    pt3.x = templateCenterX+templateDetector_->getTemplateWidth()/2;
	    pt3.y = templateCenterY+templateDetector_->getTemplateHeight()/2;
	    pt4.x = templateCenterX-templateDetector_->getTemplateWidth()/2;
	    pt4.y = templateCenterY+templateDetector_->getTemplateHeight()/2;

	    //if (showRectangle)
	    {
		    cvLine(colorImage,pt1,pt2,CV_RGB(0,0,0),3);
		    cvLine(colorImage,pt2,pt3,CV_RGB(0,0,0),3);
		    cvLine(colorImage,pt3,pt4,CV_RGB(0,0,0),3);
		    cvLine(colorImage,pt4,pt1,CV_RGB(0,0,0),3);

		    cvLine(colorImage,pt1,pt2,CV_RGB(255,255,0),1);
		    cvLine(colorImage,pt2,pt3,CV_RGB(255,255,0),1);
		    cvLine(colorImage,pt3,pt4,CV_RGB(255,255,0),1);
		    cvLine(colorImage,pt4,pt1,CV_RGB(255,255,0),1);
	    }
    }


    // create template if right mouse button has been pressed
    if (templateCenterX != -1 && templateCenterY != -1 && mouseEvent == 2)
    {
      const int regionWidth = templateDetector_->getSamplingSize ();
      const int regionHeight = templateDetector_->getSamplingSize ();
      const int templateHorizontalSamples = templateDetector_->getTemplateWidth () / templateDetector_->getSamplingSize ();
      const int templateVerticalSamples = templateDetector_->getTemplateHeight () / templateDetector_->getSamplingSize ();
      
      //unsigned char * templateData = new unsigned char[templateHorizontalSamples * templateVerticalSamples];
      unsigned char * templateData = reinterpret_cast<unsigned char*>(_mm_malloc(sizeof(unsigned char)*templateHorizontalSamples * templateVerticalSamples, 16));

      const int horizontalSamples = imageWidth / templateDetector_->getSamplingSize ();


      // copy data
      int maxResponse = 0;
      {
        for (int rowIndex = 0; rowIndex < templateVerticalSamples; ++rowIndex)
        {
          for (int colIndex = 0; colIndex < templateHorizontalSamples; ++colIndex)
          {
            const int tmpColIndex = colIndex + (templateCenterX-templateDetector_->getTemplateWidth ()/2)/regionWidth;
            const int tmpRowIndex = rowIndex + (templateCenterY-templateDetector_->getTemplateHeight ()/2)/regionHeight;
            
            templateData[rowIndex*templateHorizontalSamples + colIndex] = dataMsg->data[tmpRowIndex*horizontalSamples + tmpColIndex];
            
            if (dataMsg->data[tmpRowIndex*horizontalSamples + tmpColIndex] != 0)
            {
              ++maxResponse;
            }
          }
        }
      }
      
      
      // add template
//      if (maxResponse >= 100)
      {
        const int templateWidth = templateDetector_->getTemplateWidth ();
        const int templateHeight = templateDetector_->getTemplateHeight ();
        
        const int templateStartX = templateCenterX-templateWidth/2;
        const int templateStartY = templateCenterY-templateHeight/2;
        
        templateDetector_->addNewTemplate (templateData, 0, maxResponse, cvRect (templateStartX, templateStartY, templateWidth, templateHeight));
      
        cvSetImageROI (smoothedImage, cvRect (templateStartX, templateStartY, templateDetector_->getTemplateWidth (), templateDetector_->getTemplateHeight ()));
        templateDetector_->addContour (smoothedImage);
        cvResetImageROI (smoothedImage);
      }
      
      ROS_INFO ("template added");
      
      //delete[] templateData;
      _mm_free(templateData);
      
      templateDetector_->clearClusters ();
      templateDetector_->clusterHeuristically (4);
    }
    
    
    // copy data
    const int horizontalSamples = dataMsg->horizontalSamples;
    const int verticalSamples = dataMsg->verticalSamples;
    
    //unsigned char * data = new unsigned char[horizontalSamples*verticalSamples];
    unsigned char * data = reinterpret_cast<unsigned char*>(_mm_malloc(sizeof(unsigned char)*horizontalSamples*verticalSamples, 16));
    
    int elementIndex = 0;
    for (std::vector<unsigned char>::const_iterator iter = dataMsg->data.begin (); 
    iter != dataMsg->data.end (); 
    ++iter)
    {
      data[elementIndex] = *iter;
      
      ++elementIndex;
    }
    
    
    // detect templates
    const int threshold = threshold_;
    std::list< ::ftd::Candidate* > * candidateList = templateDetector_->process(data, threshold, dataMsg->horizontalSamples, dataMsg->verticalSamples);


    if (candidateList != NULL)
    {
		  for (int classIndex = 0; classIndex < templateDetector_->getNumOfClasses(); ++classIndex)
		  {			
			  for (std::list< ::ftd::Candidate* >::iterator candidateIter = candidateList[classIndex].begin(); candidateIter != candidateList[classIndex].end(); ++candidateIter)
			  {
					const int regionStartX = (*candidateIter)->getCol ();
					const int regionStartY = (*candidateIter)->getRow ();
					const int regionWidth = templateDetector_->getTemplateWidth();
					const int regionHeight = templateDetector_->getTemplateHeight();

					/*cvLine (
					  colorImage,
					  cvPoint (regionStartX, regionStartY),
					  cvPoint (regionStartX+regionWidth, regionStartY),
					  CV_RGB (0, 0, 255),
		        2 );
					cvLine (
					  colorImage,
					  cvPoint (regionStartX+regionWidth, regionStartY),
					  cvPoint (regionStartX+regionWidth, regionStartY+regionHeight),
					  CV_RGB (0, 0, 255),
		        2 );
					cvLine (
					  colorImage,
					  cvPoint (regionStartX+regionWidth, regionStartY+regionHeight),
					  cvPoint (regionStartX, regionStartY+regionHeight),
					  CV_RGB (0, 0, 255),
		        2 );
					cvLine (
					  colorImage,
					  cvPoint (regionStartX, regionStartY+regionHeight),
					  cvPoint (regionStartX, regionStartY),
					  CV_RGB (0, 0, 255),
		        2 );*/
		        
			  }
			  
			  // find best candidate
			  int bestCandidateResponse = 0;
			  std::list< ::ftd::Candidate* >::iterator bestCandidateIter = candidateList[classIndex].end();
			  for (std::list< ::ftd::Candidate* >::iterator candidateIter = candidateList[classIndex].begin(); candidateIter != candidateList[classIndex].end(); ++candidateIter)
			  {
			    if ((*candidateIter)->getMatchingResponse () > bestCandidateResponse)
			    {
			      bestCandidateResponse = (*candidateIter)->getMatchingResponse ();
			      bestCandidateIter = candidateIter;
			    }
			  }
			  
	      if (bestCandidateIter != candidateList[classIndex].end())
        {
				  const int regionStartX = (*bestCandidateIter)->getCol ();
				  const int regionStartY = (*bestCandidateIter)->getRow ();
				  const int regionWidth = templateDetector_->getTemplateWidth();
				  const int regionHeight = templateDetector_->getTemplateHeight();
          
				  cvLine (
				    colorImage,
				    cvPoint (regionStartX, regionStartY),
				    cvPoint (regionStartX+regionWidth, regionStartY),
				    CV_RGB (0, 255, 0),
	          2 );
				  cvLine (
				    colorImage,
				    cvPoint (regionStartX+regionWidth, regionStartY),
				    cvPoint (regionStartX+regionWidth, regionStartY+regionHeight),
				    CV_RGB (0, 255, 0),
	          2 );
				  cvLine (
				    colorImage,
				    cvPoint (regionStartX+regionWidth, regionStartY+regionHeight),
				    cvPoint (regionStartX, regionStartY+regionHeight),
				    CV_RGB (0, 255, 0),
	          2 );
				  cvLine (
				    colorImage,
				    cvPoint (regionStartX, regionStartY+regionHeight),
				    cvPoint (regionStartX, regionStartY),
				    CV_RGB (0, 255, 0),
	          2 );				  
	        
	        cvLine (
				    colorImage2,
				    cvPoint (regionStartX, regionStartY),
				    cvPoint (regionStartX+regionWidth, regionStartY),
				    CV_RGB (0, 255, 0),
	          2 );
				  cvLine (
				    colorImage2,
				    cvPoint (regionStartX+regionWidth, regionStartY),
				    cvPoint (regionStartX+regionWidth, regionStartY+regionHeight),
				    CV_RGB (0, 255, 0),
	          2 );
				  cvLine (
				    colorImage2,
				    cvPoint (regionStartX+regionWidth, regionStartY+regionHeight),
				    cvPoint (regionStartX, regionStartY+regionHeight),
				    CV_RGB (0, 255, 0),
	          2 );
				  cvLine (
				    colorImage2,
				    cvPoint (regionStartX, regionStartY+regionHeight),
				    cvPoint (regionStartX, regionStartY),
				    CV_RGB (0, 255, 0),
	          2 );
        }
			  
			  
			  // if best response is below learning threshold then learn a new template at this position
	      if ( bestCandidateIter != candidateList[classIndex].end()
	        && bestCandidateResponse < learnThreshold_ )
	      {
	        const int startX = (*bestCandidateIter)->getCol ();
	        const int startY = (*bestCandidateIter)->getRow ();

          const int regionWidth = templateDetector_->getSamplingSize ();
          const int regionHeight = templateDetector_->getSamplingSize ();
          const int templateHorizontalSamples = templateDetector_->getTemplateWidth () / templateDetector_->getSamplingSize ();
          const int templateVerticalSamples = templateDetector_->getTemplateHeight () / templateDetector_->getSamplingSize ();

          //unsigned char * templateData = new unsigned char[templateHorizontalSamples * templateVerticalSamples];
          unsigned char * templateData = reinterpret_cast<unsigned char*>(_mm_malloc(sizeof(unsigned char)*templateHorizontalSamples * templateVerticalSamples, 16));

          const int horizontalSamples = imageWidth / templateDetector_->getSamplingSize ();


          // copy data
          int maxResponse = 0;
          for (int rowIndex = 0; rowIndex < templateVerticalSamples; ++rowIndex)
          {
            for (int colIndex = 0; colIndex < templateHorizontalSamples; ++colIndex)
            {
              const int tmpColIndex = colIndex + startX/regionWidth;
              const int tmpRowIndex = rowIndex + startY/regionHeight;
              
              templateData[rowIndex*templateHorizontalSamples + colIndex] = dataMsg->data[tmpRowIndex*horizontalSamples + tmpColIndex];

              if (dataMsg->data[tmpRowIndex*horizontalSamples + tmpColIndex] != 0)
              {
                ++maxResponse;
              }
            }
          }
          
          
          // add template
          const int templateWidth = templateDetector_->getTemplateWidth ();
          const int templateHeight = templateDetector_->getTemplateHeight ();
          
          const int templateStartX = templateCenterX-templateWidth/2;
          const int templateStartY = templateCenterY-templateHeight/2;
          
          templateDetector_->addNewTemplate (templateData, 0, maxResponse, cvRect (templateStartX, templateStartY, templateWidth, templateHeight));
        
          cvSetImageROI (smoothedImage, cvRect (templateStartX, templateStartY, templateDetector_->getTemplateWidth (), templateDetector_->getTemplateHeight ()));
          templateDetector_->addContour (smoothedImage);
          cvResetImageROI (smoothedImage);


          ROS_INFO ("template added");
          
          //delete[] templateData;
          _mm_free(templateData);
          
          templateDetector_->clearClusters ();
          templateDetector_->clusterHeuristically (4);
	      }
			}
			
		  for (int classIndex = 0; classIndex < templateDetector_->getNumOfClasses(); ++classIndex)
		  {
			  ::ftd::emptyPointerList(candidateList[classIndex]);
		  }
		}

    ROS_INFO("NumOfTemplates: %d", templateDetector_->getNumOfTemplates ()); 
    
    std::stringstream ss;
    
    ss << "image" << counter_ << ".png";
    ++counter_;
    
    //cvSaveImage(ss.str().c_str(), colorImage2);
    //cvReleaseImage(&colorImage2);
    

    templateDetector_->save (fileName_.c_str ());
    
    
    CvPoint p1 = cvPoint (roi_.x, roi_.y);
    CvPoint p2 = cvPoint (roi_.x + roi_.width, roi_.y + roi_.height);    
    cvRectangle (colorImage, p1, p2, CV_RGB (255, 128, 128), 3, 8, 0);     
 
 
    // display image
    cvScale (colorImage, colorImage, 1.0/255.0);
    cvShowImage ("Image Window", colorImage);
    /*int pressedKey = */cvWaitKey (10);
    



    cvReleaseImage (&colorImage);
    cvReleaseImage (&colorImage);
    cvReleaseImage (&grayImage);
    cvReleaseImage (&smoothedImage);
    
    //delete[] data;
    _mm_free(data);
  }
  

  
private:

  int threshold_;
  int learnThreshold_;
  
  int counter_;
  
  ::ros::Subscriber dataSubscriber_;
  ::sensor_msgs::CvBridge bridge_;
  
  ::ftd::FastTemplateDetectorVS * templateDetector_;
  
  ::io::Mouse mouse_;
  
  ::std::string fileName_;
  
  CvRect roi_;
  
 
};



int
main(
  int argc,
  char ** argv )
{
  ::ros::init (argc, argv, "template_learner");
  ::ros::NodeHandle nodeHandle ("~");
      
  ::message_filters::Subscriber< ::sensor_msgs::Image > image_sub(nodeHandle, "/image", 20);
  ::message_filters::Subscriber< ::fast_template_detector::DiscretizedData > data_sub(nodeHandle, "/discretized_data", 20);
  
  ::message_filters::Synchronizer<SyncPolicy> sync (SyncPolicy(20), image_sub, data_sub);
  sync.connectInput (image_sub, data_sub);
  
  AdaptiveTemplateLearner learner (nodeHandle, (argc > 1) ? argv[1] : "test.ftd", (argc > 2) ? atoi (argv[2]) : 80, (argc > 3) ? atoi (argv[3]) : 95, (argc > 4) ? argv[4] : "raw", sync);
  
  ::ros::spin ();
}


