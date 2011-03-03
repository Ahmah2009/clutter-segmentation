#include <ftd/learners/bimodal_adaptive_template_learner.h>

#include <ftd/bump_hunting.h>

#include <ftd/discretizers/gradient_discretizer.h>
#include <ftd/discretizers/surface_normal_discretizer.h>
#include <ftd/discretizers/masked_disparity_discretizer.h>

//#include "pcl/compute_normals.h"
#include "ftd/pcl_processing.h"
//#include <pcl/io/pcd_io.h>

#include <pcl_point_cloud2_image_color/point_cloud2_image_color.h>

#define USE_GRADIENTS
//#define USE_SURFACE_NORMALS

#define USE_RESTRICTED_NUM_OF_GRADIENTS

typedef pcl_point_cloud2_image_color::PointCloud2ImageColorROS PointCloud2ImageColor;

typedef sensor_msgs::CameraInfo CameraInfo;
typedef CameraInfo::Ptr CameraInfoPtr;
typedef CameraInfo::ConstPtr CameraInfoConstPtr;

typedef sensor_msgs::Image Image;
typedef Image::Ptr ImagePtr;
typedef Image::ConstPtr ImageConstPtr;

typedef pcl::PointXYZRGB Point;
typedef pcl::PointCloud<Point> PointCloud;
typedef PointCloud::Ptr PointCloudPtr;
typedef PointCloud::ConstPtr PointCloudConstPtr;






void getBinMask( const ::cv::Mat& comMask, ::cv::Mat& binMask )
{
    if( comMask.empty() || comMask.type()!=CV_8UC1 )
        CV_Error( CV_StsBadArg, "comMask is empty or has incorrect type (not CV_8UC1)" );
    if( binMask.empty() || binMask.rows!=comMask.rows || binMask.cols!=comMask.cols )
        binMask.create( comMask.size(), CV_8UC1 );
    binMask = comMask & 1;
}



// ----------------------------------------------------------------------------
ftd::learners::
BimodalAdaptiveTemplateLearner::
BimodalAdaptiveTemplateLearner (
  const int templateWidth,
  const int templateHeight,
  const int regionSize,
  const int numOfCharsPerElement )
{
  const int templateHorizontalSamples = templateWidth/regionSize;
  const int templateVerticalSamples = templateHeight/regionSize;
  
  templateDetector1_ = new ::ftd::FastTemplateDetectorVS (templateHorizontalSamples, templateVerticalSamples, regionSize, numOfCharsPerElement, 10);
  templateDetector2_ = new ::ftd::FastTemplateDetectorVS (templateHorizontalSamples, templateVerticalSamples, regionSize, numOfCharsPerElement, 10);
  templateDetector1_->addNewClass ();
  templateDetector2_->addNewClass ();
  
#ifdef SAVE_IMAGES
  imageCounter_ = 0;
#endif // SAVE_IMAGES  
}


// ----------------------------------------------------------------------------
ftd::learners::
BimodalAdaptiveTemplateLearner::
~BimodalAdaptiveTemplateLearner ()
{
  delete templateDetector1_;
  delete templateDetector2_;
}


// ----------------------------------------------------------------------------
/*void
ftd::learners::
BimodalAdaptiveTemplateLearner::
processNextFrame (
  std::vector<unsigned char> & discretizedData,
  const int regionWidth,
  const int regionHeight,
  const int horizontalSamples,
  const int verticalSamples,
  const int threshold_,
  const int learnThreshold_ )
{   
  // copy data
  unsigned char * data = new unsigned char[discretizedData.size ()];
  
  int elementIndex = 0;
  for (std::vector<unsigned char>::const_iterator iter = discretizedData.begin (); 
  iter != discretizedData.end (); 
  ++iter)
  {
    data[elementIndex] = *iter;
    
    ++elementIndex;
  }


  // detect templates
  const int threshold = threshold_;
  std::list< ::ftd::Candidate* > * candidateList = templateDetector1_->process(data, threshold, horizontalSamples, verticalSamples);


  // process candidates
  if (candidateList != NULL)
  {
	  for (int classIndex = 0; classIndex < templateDetector1_->getNumOfClasses(); ++classIndex)
	  {			
		  for (std::list< ::ftd::Candidate* >::iterator candidateIter = candidateList[classIndex].begin(); candidateIter != candidateList[classIndex].end(); ++candidateIter)
		  {
				const int regionStartX = (*candidateIter)->getCol ();
				const int regionStartY = (*candidateIter)->getRow ();
				const int regionWidth = templateDetector1_->getTemplateWidth();
				const int regionHeight = templateDetector1_->getTemplateHeight();

        const float response = static_cast<float> ((*candidateIter)->getMatchingResponse ());
           
				/*cvLine (
				  resizedImage,
				  cvPoint (regionStartX, regionStartY),
				  cvPoint (regionStartX+regionWidth, regionStartY),
				  CV_RGB (0, 0, 255),
	        2 );
				cvLine (
				  resizedImage,
				  cvPoint (regionStartX+regionWidth, regionStartY),
				  cvPoint (regionStartX+regionWidth, regionStartY+regionHeight),
				  CV_RGB (0, 0, 255),
	        2 );
				cvLine (
				  resizedImage,
				  cvPoint (regionStartX+regionWidth, regionStartY+regionHeight),
				  cvPoint (regionStartX, regionStartY+regionHeight),
				  CV_RGB (0, 0, 255),
	        2 );
				cvLine (
				  resizedImage,
				  cvPoint (regionStartX, regionStartY+regionHeight),
				  cvPoint (regionStartX, regionStartY),
				  CV_RGB (0, 0, 255),
	        2 );*/
	        /*
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
			  const int regionWidth = templateDetector1_->getTemplateWidth();
			  const int regionHeight = templateDetector1_->getTemplateHeight();
        
			  /*cvLine (
			    resizedImage,
			    cvPoint (regionStartX, regionStartY),
			    cvPoint (regionStartX+regionWidth, regionStartY),
			    CV_RGB (0, 255, 0),
          2 );
			  cvLine (
			    resizedImage,
			    cvPoint (regionStartX+regionWidth, regionStartY),
			    cvPoint (regionStartX+regionWidth, regionStartY+regionHeight),
			    CV_RGB (0, 255, 0),
          2 );
			  cvLine (
			    resizedImage,
			    cvPoint (regionStartX+regionWidth, regionStartY+regionHeight),
			    cvPoint (regionStartX, regionStartY+regionHeight),
			    CV_RGB (0, 255, 0),
          2 );
			  cvLine (
			    resizedImage,
			    cvPoint (regionStartX, regionStartY+regionHeight),
			    cvPoint (regionStartX, regionStartY),
			    CV_RGB (0, 255, 0),
          2 );*/
          /*
      }
		  
		  
		  // if best response is below learning threshold then learn a new template at this position
      if ( bestCandidateIter != candidateList[classIndex].end()
        && bestCandidateResponse < learnThreshold_ )
      {
        const int startX = (*bestCandidateIter)->getCol ();
        const int startY = (*bestCandidateIter)->getRow ();

        const int regionWidth = templateDetector1_->getSamplingSize ();
        const int regionHeight = templateDetector1_->getSamplingSize ();
        const int templateHorizontalSamples = templateDetector1_->getTemplateWidth () / templateDetector1_->getSamplingSize ();
        const int templateVerticalSamples = templateDetector1_->getTemplateHeight () / templateDetector1_->getSamplingSize ();
        unsigned char * templateData = new unsigned char[templateHorizontalSamples * templateVerticalSamples];
        
        for (int rowIndex = 0; rowIndex < templateVerticalSamples; ++rowIndex)
        {
          for (int colIndex = 0; colIndex < templateHorizontalSamples; ++colIndex)
          {
            const int tmpColIndex = colIndex + startX/regionWidth;
            const int tmpRowIndex = rowIndex + startY/regionHeight;
            
            templateData[rowIndex*templateHorizontalSamples + colIndex] = discretizedData[tmpRowIndex*horizontalSamples + tmpColIndex];
          }
        }
                
        templateDetector1_->addNewTemplate (templateData, classIndex);
        
        delete[] templateData;
        
        templateDetector1_->clearClusters ();
        templateDetector1_->clusterHeuristically (4);
      }
		}
		
	  for (int classIndex = 0; classIndex < templateDetector1_->getNumOfClasses(); ++classIndex)
	  {
		  ::ftd::emptyPointerList(candidateList[classIndex]);
	  }
	}


  // display image
  //cvScale (resizedImage, resizedImage, 1.0/255.0);
  //cvShowImage ("Image Window", resizedImage);
  //int pressedKey = cvWaitKey (10);


  //cvReleaseImage (&colorImage);
  //cvReleaseImage (&resizedImage);
  
  delete[] data;
}*/


// ----------------------------------------------------------------------------
/*void
ftd::learners::
BimodalAdaptiveTemplateLearner::
addTemplate (
  std::vector<unsigned char> & discretizedData,
  const int templateCenterX,
  const int templateCenterY,
  const int regionWidth,
  const int regionHeight,
  const int horizontalSamples,
  const int verticalSamples,
  const int classIndex )
{    
  const int startX = templateCenterX - templateDetector1_->getTemplateWidth ()/2;
  const int startY = templateCenterY - templateDetector1_->getTemplateHeight ()/2;

  const int templateHorizontalSamples = templateDetector1_->getTemplateWidth () / templateDetector1_->getSamplingSize ();
  const int templateVerticalSamples = templateDetector1_->getTemplateHeight () / templateDetector1_->getSamplingSize ();
  
  unsigned char * templateData = new unsigned char[templateHorizontalSamples * templateVerticalSamples];
  
  for (int rowIndex = 0; rowIndex < templateVerticalSamples; ++rowIndex)
  {
    for (int colIndex = 0; colIndex < templateHorizontalSamples; ++colIndex)
    {
      const int tmpColIndex = colIndex + startX/regionWidth;
      const int tmpRowIndex = rowIndex + startY/regionHeight;
      
      templateData[rowIndex*templateHorizontalSamples + colIndex] = discretizedData[tmpRowIndex*horizontalSamples + tmpColIndex];
    }
  }
          
  templateDetector1_->addNewTemplate (templateData, classIndex);
  
  delete[] templateData;
  
  templateDetector1_->clearClusters ();
  templateDetector1_->clusterHeuristically (4);
}*/

// ----------------------------------------------------------------------------
void
ftd::learners::
BimodalAdaptiveTemplateLearner::
loadTemplates (
  std::string & fileName1,
  std::string & fileName2 )
{ 
  templateDetector1_->load (fileName1.c_str ());
  //templateDetector2_->load (fileName2.c_str ());
  

}


// ----------------------------------------------------------------------------
void
ftd::learners::
BimodalAdaptiveTemplateLearnerBAG::
loadTemplates (
  std::string & fileName1,
  std::string & fileName2 )
{ 
  templateDetector1_->load (fileName1.c_str ());
  //templateDetector2_->load (fileName2.c_str ());
  
  
  std::ifstream fileStream (
    fileName2.c_str (),
    std::ifstream::in | std::ifstream::binary );
    
  if (fileStream.fail ())
  {
    std::cerr << "could not open file for reading" << std::endl;
    return;
  }
  
   
    
  int numOfTemplates = 0;
  fileStream.read (reinterpret_cast<char*> (&numOfTemplates), sizeof (numOfTemplates));

  for (unsigned int templateIndex = 0; templateIndex < numOfTemplates; ++templateIndex)
  {
    IplImage * sobelDx = templateDetector1_->readImage (fileStream);
    IplImage * sobelDy = templateDetector1_->readImage (fileStream);
    IplImage * sobelMagnitude = templateDetector1_->readImage (fileStream);
    IplImage * disparity = templateDetector1_->readImage (fileStream);
    
    sobelDxTemplates_.push_back (sobelDx);
    sobelDyTemplates_.push_back (sobelDy);
    sobelMagnitudeTemplates_.push_back (sobelMagnitude);
    disparityTemplates_.push_back (disparity);
  }  
  
  
  fileStream.close ();
}


// ----------------------------------------------------------------------------
void
ftd::learners::
BimodalAdaptiveTemplateLearner::
saveTemplates (
  std::string & fileName1,
  std::string & fileName2 )
{ 
  templateDetector1_->save (fileName1.c_str ());
  //templateDetector2_->save (fileName2.c_str ());
  

}


// ----------------------------------------------------------------------------
void
ftd::learners::
BimodalAdaptiveTemplateLearnerBAG::
saveTemplates (
  std::string & fileName1,
  std::string & fileName2 )
{ 
  templateDetector1_->save (fileName1.c_str ());
  //templateDetector2_->save (fileName2.c_str ());
  
  std::ofstream fileStream (
    fileName2.c_str (),
    std::ofstream::out | std::ofstream::binary );
    
  if (fileStream.fail ())
  {
    std::cerr << "could not open file for writing" << std::endl;
    return;
  }
  
  int numOfTemplates = disparityTemplates_.size ();
  fileStream.write (reinterpret_cast<char*> (&numOfTemplates), sizeof (numOfTemplates));

  for (unsigned int templateIndex = 0; templateIndex < numOfTemplates; ++templateIndex)
  {
    templateDetector1_->write (fileStream, sobelDxTemplates_[templateIndex]);
    templateDetector1_->write (fileStream, sobelDyTemplates_[templateIndex]);
    templateDetector1_->write (fileStream, sobelMagnitudeTemplates_[templateIndex]);
    templateDetector1_->write (fileStream, disparityTemplates_[templateIndex]);
    
    writePointCloud(fileStream, pointCloudTemplates_[templateIndex]);
  }
  
  fileStream.close ();
}


// ----------------------------------------------------------------------------
ftd::learners::
BimodalAdaptiveTemplateLearnerBAG::
BimodalAdaptiveTemplateLearnerBAG (
  std::string & bagFileName,
  std::string & imageTopic,
  std::string & camInfoTopic,
  std::string & discretizedDataTopic,
  std::string & cloudTopic,
  std::string & disparityTopic,
  std::string & templateFileName1,
  std::string & templateFileName2,
  const int templateWidth,
  const int templateHeight,
  const int regionSize,
  const int numOfCharsPerElement,
  const int detectionThreshold,
  const int learningThreshold )
: BimodalAdaptiveTemplateLearner (templateWidth, templateHeight, regionSize, numOfCharsPerElement),
  sync_ (AdaptiveTemplateLearnerBAGSyncPolicy (200)),
  processed_ (false),
  stop_ (false),
  templateFileName1_ (templateFileName1),
  templateFileName2_ (templateFileName2),
  detectionThreshold_ (detectionThreshold),
  learningThreshold_ (learningThreshold)
{
  std::cerr << "thresholds: " << detectionThreshold_ << ", " << learningThreshold_ << std::endl;

  ROS_INFO ("Reading data from %s", bagFileName.c_str ());
  if (!player_.open (bagFileName, ros::Time ()))
  {
    std::cerr << "Error while opening bag-file." << std::endl;
    return;
  }

  std::cerr << "thresholds: " << detectionThreshold_ << ", " << learningThreshold_ << std::endl;

  
  imageReceived_ = false;
  cloudReceived_ = false;
  camInfoReceived_ = false;
  disparityReceived_ = false;

  player_.addHandler< ::sensor_msgs::Image > (imageTopic, &BimodalAdaptiveTemplateLearnerBAG::handlerImage, this);
  player_.addHandler< ::sensor_msgs::CameraInfo > (camInfoTopic, &BimodalAdaptiveTemplateLearnerBAG::handlerCamInfo, this);
  player_.addHandler< ::sensor_msgs::PointCloud2 > (cloudTopic, &BimodalAdaptiveTemplateLearnerBAG::handlerPointCloud2, this);
  player_.addHandler< ::stereo_msgs::DisparityImage > (disparityTopic, &BimodalAdaptiveTemplateLearnerBAG::handlerDisparityImage, this);

  sync_.registerCallback (boost::bind (&BimodalAdaptiveTemplateLearnerBAG::callback2, this, _1, _2, _3, _4));
  init_ = true;
  
	// get camera info message
  if (!nextMsg ())
		ROS_ERROR("Message not found in bag file.\n");
}


// ----------------------------------------------------------------------------
ftd::learners::
BimodalAdaptiveTemplateLearnerBAG::
~BimodalAdaptiveTemplateLearnerBAG ()
{
  saveTemplates (templateFileName1_, templateFileName2_);
  ROS_INFO("Templates saved");
  
  player_.close ();
  ROS_INFO("Player closed");
}


// ----------------------------------------------------------------------------
CvRect
ftd::learners::
BimodalAdaptiveTemplateLearnerBAG::
estimateROI (
  IplImage * smoothedImage,
  CvRect & initialRegion )
{
  // compute gradients
  IplImage * sobelDx = cvCreateImage (cvGetSize (smoothedImage), IPL_DEPTH_32F, 1);
  IplImage * sobelDy = cvCreateImage (cvGetSize (smoothedImage), IPL_DEPTH_32F, 1);
  IplImage * sobelMagnitude = cvCreateImage (cvGetSize (smoothedImage), IPL_DEPTH_32F, 1);
  IplImage * sobelAngle = cvCreateImage (cvGetSize (smoothedImage), IPL_DEPTH_32F, 1);

  cvSobel (smoothedImage, sobelDx, 1, 0, 3);
  cvSobel (smoothedImage, sobelDy, 0, 1, 3);

  cvCartToPolar (sobelDx, sobelDy, sobelMagnitude, sobelAngle, 1); // the "1" means that the angles are in degree
  
  //CvRect initialRegion = cvRect(0, 0, smoothedImage->width, smoothedImage->height);

  IplImage * mask = cvCreateImage (cvGetSize (smoothedImage), IPL_DEPTH_8U, 1);
  cvSetZero(mask);
  
  for (int rowIndex = 0; rowIndex < sobelMagnitude->height; ++rowIndex)
  {
    for (int colIndex = 0; colIndex < sobelMagnitude->width; ++colIndex)
    {
      if (CV_IMAGE_ELEM (sobelMagnitude, float, rowIndex, colIndex) > 30.0f)
      {
        CV_IMAGE_ELEM (mask, unsigned char, rowIndex, colIndex) = 255;
      }
    }
  }
  
  cvShowImage("mask", mask);
  cvWaitKey(10);
  
  float allowableLossFraction = 0.15, peelFraction = 0.3;
  float mean;
  CvRect finalRegion = BumpHunting::gradientBumpHunting (
    mask,
    initialRegion,
    mean, 
    allowableLossFraction, 
    peelFraction, 
    0.2 );
  
  //CvPoint p1 = cvPoint (finalRegion.x, finalRegion.y);
  //CvPoint p2 = cvPoint (finalRegion.x + finalRegion.width, finalRegion.y + finalRegion.height);

  //cvRectangle (colorImage, p1, p2, CV_RGB (255, 128, 128), 3, 8, 0);
      
  cvReleaseImage(&sobelDx);
  cvReleaseImage(&sobelDy);
  cvReleaseImage(&sobelMagnitude);
  cvReleaseImage(&sobelAngle);
  cvReleaseImage(&mask);
  
  return finalRegion;
}


// ----------------------------------------------------------------------------
IplImage *
ftd::learners::
BimodalAdaptiveTemplateLearnerBAG::
segmentObject (
  IplImage * colorImage,
  IplImage * dilatedMask,
  IplImage * erodedMask,
  CvRect & roi )
{
  using ::cv::Mat;
  
  //IplImage * dilatedMask = cvCreateImage (cvGetSize (roughMask), IPL_DEPTH_8U, 1);
  //IplImage * erodedMask = cvCreateImage (cvGetSize (roughMask), IPL_DEPTH_8U, 1);
  
  //cvDilate(roughMask, dilatedMask, NULL, 5);
  //cvErode(roughMask, erodedMask, NULL, 10);
  
  IplImage * colorImage8U = cvCreateImage (cvGetSize (colorImage), IPL_DEPTH_8U, 3);
  cvConvert (colorImage, colorImage8U);
  
  IplImage * objectMask = cvCreateImage (cvGetSize (colorImage), IPL_DEPTH_8U, 1);
  cvSetZero (objectMask);

  //std::cerr << "test1" << std::endl;
  //for (size_t i = 0; i < perceptionData.roi.size (); ++i)
  {
    const Mat tmpImage(colorImage8U);
    Mat mask;
    mask.create(tmpImage.size(), CV_8UC1);
    Mat bgdModel, fgdModel;
    
    //std::cerr << "test2" << std::endl;
    mask.setTo(::cv::Scalar::all(::cv::GC_BGD));
    
    ::cv::Rect rect;
    rect.x = roi.x;
    rect.y = roi.y;
    rect.width = roi.width;
    rect.height = roi.height;
    
    rect.x = max(0, rect.x);
    rect.y = max(0, rect.y);
    rect.width = min(rect.width, tmpImage.cols-rect.x);
    rect.height = min(rect.height, tmpImage.rows-rect.y);
    (mask(rect)).setTo( ::cv::Scalar(::cv::GC_PR_BGD) );
    
    //std::cerr << "test3" << std::endl;
    IplImage * tmpMask = cvCreateImage (cvGetSize (erodedMask), IPL_DEPTH_8U, 1);
    cvSetZero (tmpMask);
    
    for (int rowIndex = rect.y; rowIndex < (rect.y+rect.height); ++rowIndex)
    {
      for (int colIndex = rect.x; colIndex < (rect.x+rect.width); ++colIndex)
      {
        if (CV_IMAGE_ELEM (erodedMask, unsigned char, rowIndex, colIndex) != 0)
        {
          ::cv::circle( mask, ::cv::Point(colIndex, rowIndex), 0, ::cv::GC_FGD, 1 );
          //mask(rowIndex, colIndex) = GC_PR_FGD;
          CV_IMAGE_ELEM (tmpMask, unsigned char, rowIndex, colIndex) = 255;
        }
        else if (CV_IMAGE_ELEM (dilatedMask, unsigned char, rowIndex, colIndex) == 0)
        {
          ::cv::circle( mask, ::cv::Point(colIndex, rowIndex), 0, ::cv::GC_BGD, 1 );
          //mask(rowIndex, colIndex) = GC_PR_BGD;
          CV_IMAGE_ELEM (tmpMask, unsigned char, rowIndex, colIndex) = 128;
        }
      }
    }
    
    //std::cerr << "test4" << std::endl;
    //cvShowImage("tmpMask", tmpMask);
    //cvReleaseImage(&tmpMask);
    
    Mat res;
    Mat binMask;

    //std::cerr << "test5" << std::endl;
    //getBinMask( mask, binMask );
    //tmpImage.copyTo( res, binMask );
    

    //::cv::imshow ("res1", res);

    grabCut (tmpImage, mask, rect, bgdModel, fgdModel, ::cv::GC_INIT_WITH_MASK);
    //grabCut (tmpImage, mask, rect, bgdModel, fgdModel, 1);
    



    //std::cerr << "test6" << std::endl;
    getBinMask( mask, binMask );

    for (int rowIndex = 0; rowIndex < objectMask->height; ++rowIndex)
    {
      for (int colIndex = 0; colIndex < objectMask->width; ++colIndex)
      {
        if (binMask.at<unsigned char> (rowIndex, colIndex) != 0)
        {
          CV_IMAGE_ELEM (objectMask, unsigned char, rowIndex, colIndex) = 255;
        }
      }
    }
    
    tmpImage.copyTo( res, binMask );  
    //std::cerr << "test7" << std::endl;
    ::cv::imshow ("res", res);
  }
  
//  cvReleaseImage (&dilatedMask);
//  cvReleaseImage (&erodedMask);
  cvReleaseImage (&colorImage8U);
  
  //cvShowImage ("refinedObjectMask", objectMask);
  //cvWaitKey (10);
  
  return objectMask;
}


// ----------------------------------------------------------------------------
int
ftd::learners::
BimodalAdaptiveTemplateLearnerBAG::
evaluateMaskedDisparity (
  const int templateId,
  const int templateStartX,
  const int templateStartY,
  const int templateHorizontalSamples,
  const int templateVerticalSamples,
  const int regionWidth,
  const int regionHeight,
  const int horizontalSamples,
  const int verticalSamples,
  IplImage * colorImageClone,
  const ::stereo_msgs::DisparityImageConstPtr & disparityMsg )
{
  unsigned char * templateDisparityData = new unsigned char[templateHorizontalSamples * templateVerticalSamples];
  
  
  //segment object
  /*std::vector<unsigned char> maskedDisparityData (horizontalSamples * verticalSamples, 0);
  {
    const int templateWidth = templateDetector1_->getTemplateWidth ();
    const int templateHeight = templateDetector1_->getTemplateHeight ();
    
    const int templateCenterX = templateStartX + templateWidth/2;
    const int templateCenterY = templateStartY + templateHeight/2;
    
    CvRect objectROI = cvRect (templateStartX-templateWidth/2, templateStartY-templateHeight/2, 2*templateWidth, 2*templateHeight);
    
    IplImage * objectMask = cvCreateImage (cvGetSize (colorImageClone), IPL_DEPTH_8U, 1);
    IplImage * backgroundMask = cvCreateImage (cvGetSize (colorImageClone), IPL_DEPTH_8U, 1);
    cvSetZero (objectMask);
    cvSetZero (backgroundMask);
    
    for (int rowIndex = 0; rowIndex < backgroundMask->height; ++rowIndex)
    {
      for (int colIndex = 0; colIndex < backgroundMask->width; ++colIndex)
      {
        if ( colIndex > objectROI.x+templateWidth/4
          && colIndex < objectROI.x+objectROI.width-templateWidth/4
          && rowIndex > objectROI.y+templateHeight/4
          && rowIndex < objectROI.y+objectROI.height-templateHeight/4 )
        {
          CV_IMAGE_ELEM (backgroundMask, unsigned char, rowIndex, colIndex) = 255;
        }
        
        if ( colIndex > templateCenterX-templateWidth/8
          && colIndex < templateCenterX+templateWidth/8
          && rowIndex > templateCenterY-templateHeight/8
          && rowIndex < templateCenterY+templateHeight/8 )
        {
          CV_IMAGE_ELEM (objectMask, unsigned char, rowIndex, colIndex) = 255;
        }
        
      }
    }
    
    //cvShowImage ("backgroundMask", backgroundMask);
    //cvWaitKey (10);
    //cvShowImage ("objectMask", objectMask);
    //cvWaitKey (10);
    
    IplImage * refinedMask = segmentObject (colorImageClone, backgroundMask, objectMask, objectROI);
    
    // get discretized disparity data
    ::ftd::discretizers::MaskedDisparityDiscretizer discretizer;
    discretizer.discretize(
      colorImageClone,
      refinedMask,
      disparityMsg,
      regionWidth,
      regionHeight,
      maskedDisparityData );          
      
      
    cvReleaseImage (&refinedMask);
    
    cvReleaseImage (&objectMask);
    cvReleaseImage (&backgroundMask);
  }*/
  
  //segment object
    
  IplImage * resizedColorImageClone = cvCreateImage (cvSize (colorImageClone->width/2, colorImageClone->height/2), IPL_DEPTH_32F, 3);
  cvResize (colorImageClone, resizedColorImageClone);
  
  std::vector<unsigned char> maskedDisparityData (horizontalSamples * verticalSamples, 0);
  {
    const int templateWidth = (templateDetector1_->getTemplateWidth ())/2;
    const int templateHeight = (templateDetector1_->getTemplateHeight ())/2;
    
    const int templateCenterX = (templateStartX)/2 + templateWidth/2;
    const int templateCenterY = (templateStartY)/2 + templateHeight/2;
    
    CvRect objectROI = cvRect (templateStartX/2-templateWidth/2, templateStartY/2-templateHeight/2, 2*templateWidth, 2*templateHeight);
    
    IplImage * objectMask = cvCreateImage (cvGetSize (resizedColorImageClone), IPL_DEPTH_8U, 1);
    IplImage * backgroundMask = cvCreateImage (cvGetSize (resizedColorImageClone), IPL_DEPTH_8U, 1);
    cvSetZero (objectMask);
    cvSetZero (backgroundMask);
    
    for (int rowIndex = 0; rowIndex < backgroundMask->height; ++rowIndex)
    {
      for (int colIndex = 0; colIndex < backgroundMask->width; ++colIndex)
      {
        if ( colIndex > objectROI.x+templateWidth/4
          && colIndex < objectROI.x+objectROI.width-templateWidth/4
          && rowIndex > objectROI.y+templateHeight/4
          && rowIndex < objectROI.y+objectROI.height-templateHeight/4 )
        {
          CV_IMAGE_ELEM (backgroundMask, unsigned char, rowIndex, colIndex) = 255;
        }
        
        if ( colIndex > templateCenterX-templateWidth/8
          && colIndex < templateCenterX+templateWidth/8
          && rowIndex > templateCenterY-templateHeight/8
          && rowIndex < templateCenterY+templateHeight/8 )
        {
          CV_IMAGE_ELEM (objectMask, unsigned char, rowIndex, colIndex) = 255;
        }
        
      }
    }
    
    cvShowImage ("backgroundMask", backgroundMask);
    cvWaitKey (10);
    cvShowImage ("objectMask", objectMask);
    cvWaitKey (10);
    
    IplImage * refinedMask = segmentObject (resizedColorImageClone, backgroundMask, objectMask, objectROI);
    
    IplImage *resizedRefinedMask = cvCreateImage (cvGetSize (colorImageClone), IPL_DEPTH_8U, 1);
    cvResize (refinedMask, resizedRefinedMask);
    
    // get discretized disparity data
    ::ftd::discretizers::MaskedDisparityDiscretizer discretizer;
    discretizer.discretize(
      colorImageClone,
      resizedRefinedMask,
      disparityMsg,
      regionWidth,
      regionHeight,
      maskedDisparityData );          
      
      
    cvReleaseImage (&refinedMask);
    cvReleaseImage (&resizedRefinedMask);
    
    cvReleaseImage (&objectMask);
    cvReleaseImage (&backgroundMask);
  }
  
  cvReleaseImage (&resizedColorImageClone);



  for (int rowIndex = 0; rowIndex < templateVerticalSamples; ++rowIndex)
  {
    for (int colIndex = 0; colIndex < templateHorizontalSamples; ++colIndex)
    {
      const int tmpColIndex = colIndex + templateStartX/regionWidth;
      const int tmpRowIndex = rowIndex + templateStartY/regionHeight;
      
      templateDisparityData[rowIndex*templateHorizontalSamples + colIndex] = maskedDisparityData[tmpRowIndex*horizontalSamples + tmpColIndex];
    }
  }
  
  const int response = templateDetector2_->computeTemplateResponse (templateDisparityData, templateId);
  
  delete[] templateDisparityData;
  
  return response;
}


// ----------------------------------------------------------------------------
void
ftd::learners::
BimodalAdaptiveTemplateLearnerBAG::
callback (
  const ::sensor_msgs::ImageConstPtr & imageMsg,
  const ::sensor_msgs::CameraInfoConstPtr & camInfoMsg,
  const ::sensor_msgs::PointCloud2ConstPtr & cloudMsg,
  const ::stereo_msgs::DisparityImageConstPtr & disparityMsg )
{
  using ::ftd::BumpHunting;
  using ::ftd::discretizers::GradientDiscretizer;
  using ::ftd::discretizers::SurfaceNormalDiscretizer;
  using ::cv::Mat;
  
  
  std::cerr << "callback called" << std::endl;
 
  // get OpenCV image
  IplImage * image = bridge_.imgMsgToCv(imageMsg); 


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
  
  
  // convert to gray value image
  IplImage * grayImage = NULL;
  
  if (image->depth == 32)
  {
    grayImage = cvCreateImage(cvGetSize(image), IPL_DEPTH_32F, 1);
    cvCvtColor(image, grayImage, CV_BGR2GRAY);
  }
  else if (image->depth == 8)
  {
    if (image->nChannels == 3)
    {
      IplImage * tmpGrayImage = cvCreateImage(cvGetSize(image), IPL_DEPTH_8U, 1);
      cvCvtColor(image, tmpGrayImage, CV_BGR2GRAY);
      
      grayImage = cvCreateImage(cvGetSize(image), IPL_DEPTH_32F, 1);
      cvConvert(tmpGrayImage, grayImage);
      
      cvReleaseImage(&tmpGrayImage);
    }
    else if (image->nChannels == 1)
    {
      grayImage = cvCreateImage(cvGetSize(image), IPL_DEPTH_32F, 1);
      cvConvert(image, grayImage);
    }
    else
    {
      ROS_INFO("Number of image channels is not supported!");
      return;
    }
  }
  else
  {
    ROS_INFO("Image depth not supported!");
    return;
  }
  
  IplImage * colorImageClone = static_cast<IplImage*>(cvClone(colorImage));
  
  IplImage * smoothedImage = cvCreateImage (cvGetSize (grayImage), IPL_DEPTH_32F, 1);
  cvSmooth (grayImage, smoothedImage, CV_GAUSSIAN, 5);
  
  ::ros::Time t1 = ::ros::Time::now ();
  // compute ROIs
  ::sensor_msgs::Image image_rect;
  PointCloud cloud_rgb;
  PointCloud2ImageColor::getColorPointCloud2 (cloudMsg, imageMsg, image_rect, cloud_rgb);

  PointCloudConstPtr cloud_rgb_ptr = boost::make_shared<PointCloud> (cloud_rgb);
  ImageConstPtr image_rect_ptr = boost::make_shared<Image> (image_rect);
  ROS_INFO ("Got a PointCloud<PointXYZRGB> with %d points.", (int)cloud_rgb.points.size ());
  
  dot::PerceptionData perceptionData;
  ftd::ExtractObjectRegions eor;
  //eor.setInputCloud (cloud_rgb_ptr);
  eor.setInputImage (image_rect_ptr);
  eor.setInputCameraInfo (camInfoMsg);
  eor.extract (perceptionData, true);

  for (size_t i = 0; i < perceptionData.roi.size (); ++i)
  {
    CvPoint p1 = cvPoint (perceptionData.roi[i].x_offset, perceptionData.roi[i].y_offset);
    CvPoint p2 = cvPoint (perceptionData.roi[i].x_offset + perceptionData.roi[i].width, perceptionData.roi[i].y_offset + perceptionData.roi[i].height);

    cvRectangle (colorImage, p1, p2, CV_RGB (255, 0, 0), 3, 8, 0);
  }
  ::ros::Time t2 = ::ros::Time::now ();
  

  // get disparity image
  CvMat * disparityMatrix = cvCreateMatHeader (disparityMsg->image.height, disparityMsg->image.width, CV_32FC1);
  cvSetData (disparityMatrix, static_cast<void*> (const_cast<unsigned char*> (&(disparityMsg->image.data[0]))), disparityMsg->image.step);

  ROS_INFO("Created disparity matrix"); 
  
  IplImage * disparityImage = cvCreateImage (cvSize (disparityMatrix->cols, disparityMatrix->rows), IPL_DEPTH_32F, 1);
  cvSetZero (disparityImage);

  ROS_INFO("Created disparity image"); 

  for (int rowIndex = 0; rowIndex < disparityImage->height; ++rowIndex)
  {
    for (int colIndex = 0; colIndex < disparityImage->width; ++colIndex)
    {
      const float dispValue = CV_MAT_ELEM (*disparityMatrix, float, rowIndex, colIndex);
      if (dispValue > 50.0f /*&& dispValue < 120.0f*/)
      {
        CV_IMAGE_ELEM (disparityImage, float, rowIndex, colIndex) = dispValue;
      }
    }
  }
  
  cvShowImage ("disp", disparityImage);   
  cvWaitKey (10);
  
  ROS_INFO ("Filled disparity image"); 
 
  ::ros::Time t3 = ::ros::Time::now ();
      
  ROS_INFO ("num of templates: %d", templateDetector1_->getNumOfTemplates ());
  
  // discretize data
  const int regionWidth_ = 7;
  const int regionHeight_ = 7;
  
  const int horizontalSamples = grayImage->width/regionWidth_;
  const int verticalSamples = grayImage->height/regionHeight_;
  
  unsigned char * discretizedData = new unsigned char[horizontalSamples*verticalSamples];
  
  std::vector<unsigned char> discretizedDataVec (horizontalSamples*verticalSamples);
  
  
  DiscretizationType discretizationType_ = GRADIENT;
  //DiscretizationType discretizationType_ = SURFACE_NORMAL;
  
  if (discretizationType_ == GRADIENT)
  {
    GradientDiscretizer discretizer;
    discretizer.discretize(
      smoothedImage,
      regionWidth_,
      regionHeight_,
      discretizedDataVec,
      8 );
  }
  else if (discretizationType_ == SURFACE_NORMAL)
  {
    SurfaceNormalDiscretizer discretizer;
    discretizer.discretize(
      cloudMsg,
      disparityMsg,
      regionWidth_,
      regionHeight_,
      discretizedDataVec);
  }
  else if (discretizationType_ == MASKED_DISPARITY)
  {
    // create mask
    IplImage * mask = NULL;
    
    ::ftd::discretizers::MaskedDisparityDiscretizer discretizer;
    discretizer.discretize(
      colorImage,
      mask,
      disparityMsg,
      regionWidth_,
      regionHeight_,
      discretizedDataVec );
  }    
  
  
  ::ros::Time t4 = ::ros::Time::now ();
  
    
  int elementIndex = 0;
  for (std::vector<unsigned char>::const_iterator iter = discretizedDataVec.begin (); 
  iter != discretizedDataVec.end (); 
  ++iter)
  {
    discretizedData[elementIndex] = *iter;
    
    ++elementIndex;
  }    
    
  // detect templates
  std::cerr << "threshold: " << detectionThreshold_ << std::endl;
  std::list< ::ftd::Candidate* > * candidateList = templateDetector1_->process(discretizedData, detectionThreshold_, horizontalSamples, verticalSamples);


  ::ros::Time t5 = ::ros::Time::now ();
  ROS_INFO("Computed candidates"); 

  bool objectDetected = false;
  if (candidateList != NULL)
  {
	  for (int classIndex = 0; classIndex < templateDetector1_->getNumOfClasses(); ++classIndex)
	  {			
		  for (std::list< ::ftd::Candidate* >::iterator candidateIter = candidateList[classIndex].begin(); candidateIter != candidateList[classIndex].end(); ++candidateIter)
		  {
				const int regionStartX = (*candidateIter)->getCol ();
				const int regionStartY = (*candidateIter)->getRow ();
				const int regionWidth = templateDetector1_->getTemplateWidth();//dotTemplate.getContours()[(*candidateIter)->getIndex ()-1]->width;
				const int regionHeight = templateDetector1_->getTemplateHeight();//dotTemplate.getContours()[(*candidateIter)->getIndex ()-1]->height;

        const float response = static_cast<float> ((*candidateIter)->getMatchingResponse ());
        
        //std::cerr << response << ",";
        //std::cerr << "(" << regionStartX << "," << regionStartY << ");";
        
				cvLine (
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
	        2 );
	        
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
        std::cerr << "bestCandidateResponse: " << bestCandidateResponse << std::endl;
        
        const int templateHorizontalSamples = templateDetector1_->getTemplateWidth () / templateDetector1_->getSamplingSize ();
        const int templateVerticalSamples = templateDetector1_->getTemplateHeight () / templateDetector1_->getSamplingSize ();
        
        ::ros::Time t_disp1 = ::ros::Time::now ();
        int maskedDisparityResponse = evaluateMaskedDisparity (
          (*bestCandidateIter)->getIndex ()-1,
          (*bestCandidateIter)->getCol (),
          (*bestCandidateIter)->getRow (),
          templateHorizontalSamples,
          templateVerticalSamples,
          templateDetector1_->getSamplingSize (),
          templateDetector1_->getSamplingSize (),
          horizontalSamples,
          verticalSamples,
          colorImageClone,
          disparityMsg );
        ::ros::Time t_disp2 = ::ros::Time::now ();
        
        ROS_INFO ("disp time: %f", (t_disp2-t_disp1).toSec ());

        std::cerr << "masked disparity response: " << maskedDisparityResponse << std::endl;
  

        objectDetected = true;
        
			  const int regionStartX = (*bestCandidateIter)->getCol ();
			  const int regionStartY = (*bestCandidateIter)->getRow ();
			  const int regionWidth = templateDetector1_->getTemplateWidth();//dotTemplate.getContours()[(*candidateIter)->getIndex ()-1]->width;
			  const int regionHeight = templateDetector1_->getTemplateHeight();//dotTemplate.getContours()[(*candidateIter)->getIndex ()-1]->height;
			  
			  templateDetector1_->drawContour ((*bestCandidateIter)->getIndex ()-1, colorImage, regionStartX, regionStartY);
        
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
      }
		  
		  
		  // if best response is below learning threshold then learn a new template at this position
      if ( bestCandidateIter != candidateList[classIndex].end()
        && bestCandidateResponse < learningThreshold_ )
      {
        int templateStartX = (*bestCandidateIter)->getCol ();
        int templateStartY = (*bestCandidateIter)->getRow ();

        // correct template center based on disparity
        {
          const float maxDisparityDist = 7.5f;
        
          const float centerDisparity = CV_IMAGE_ELEM (disparityImage, float, templateStartY+templateDetector1_->getTemplateHeight ()/2, templateStartX+templateDetector1_->getTemplateWidth ()/2);
          //std::cerr << "centerDisparity: " << centerDisparity << std::endl;
          
          if (centerDisparity > 1.0f)
          {
            const int startX = templateStartX;
            const int startY = templateStartY;
            
            const int endX = startX+templateDetector1_->getTemplateWidth ();
            const int endY = startY+templateDetector1_->getTemplateHeight ();
            
            int newCenterX = 0.0f;
            int newCenterY = 0.0f;
            int counter = 0;
            for (int rowIndex = startY; rowIndex < endY; ++rowIndex)
            {
              for (int colIndex = startX; colIndex < endX; ++colIndex)
              {
                if (abs(CV_IMAGE_ELEM (disparityImage, float, rowIndex, colIndex) - centerDisparity) < maxDisparityDist)
                {
                  newCenterX += colIndex;
                  newCenterY += rowIndex;
                  ++counter;
                }
                else
                {
                  CV_IMAGE_ELEM (disparityImage, float, rowIndex, colIndex) = 0;
                }
              }
            }
            
            //std::cerr << "old: " << templateStartX << ", " << templateStartY << std::endl;

            if (counter > 0)
            {
              templateStartX = newCenterX/counter-templateDetector1_->getTemplateWidth ()/2;
              //templateStartY = newCenterY/counter-templateDetector1_->getTemplateHeight ()/2;
            }

            //std::cerr << "new: " << templateStartX << ", " << templateStartY << std::endl;
          }
          
          //cvShowImage ("disp", disparityImage);
          //cvWaitKey (10);
        }
        /*discretizer.discretize(
          grayImage,
          regionWidth_,
          regionHeight_,
          discretizedData,
          8 );*/
          
        const int regionWidth = templateDetector1_->getSamplingSize ();
        const int regionHeight = templateDetector1_->getSamplingSize ();
        const int templateHorizontalSamples = templateDetector1_->getTemplateWidth () / templateDetector1_->getSamplingSize ();
        const int templateVerticalSamples = templateDetector1_->getTemplateHeight () / templateDetector1_->getSamplingSize ();
        unsigned char * templateData = new unsigned char[templateHorizontalSamples * templateVerticalSamples];
        unsigned char * templateDisparityData = new unsigned char[templateHorizontalSamples * templateVerticalSamples];
        
        
        //segment object
        ::ros::Time t_segObj1 = ::ros::Time::now ();
        std::vector<unsigned char> maskedDisparityData (horizontalSamples * verticalSamples, 0);
        /*{
          const int templateWidth = templateDetector1_->getTemplateWidth ();
          const int templateHeight = templateDetector1_->getTemplateHeight ();
          
          const int templateCenterX = templateStartX + templateWidth/2;
          const int templateCenterY = templateStartY + templateHeight/2;
          
          CvRect objectROI = cvRect (templateStartX-templateWidth/2, templateStartY-templateHeight/2, 2*templateWidth, 2*templateHeight);
          
          IplImage * objectMask = cvCreateImage (cvGetSize (colorImage), IPL_DEPTH_8U, 1);
          IplImage * backgroundMask = cvCreateImage (cvGetSize (colorImage), IPL_DEPTH_8U, 1);
          cvSetZero (objectMask);
          cvSetZero (backgroundMask);
          
          for (int rowIndex = 0; rowIndex < backgroundMask->height; ++rowIndex)
          {
            for (int colIndex = 0; colIndex < backgroundMask->width; ++colIndex)
            {
              if ( colIndex > objectROI.x+templateWidth/4
                && colIndex < objectROI.x+objectROI.width-templateWidth/4
                && rowIndex > objectROI.y+templateHeight/4
                && rowIndex < objectROI.y+objectROI.height-templateHeight/4 )
              {
                CV_IMAGE_ELEM (backgroundMask, unsigned char, rowIndex, colIndex) = 255;
              }
              
              if ( colIndex > templateCenterX-templateWidth/8
                && colIndex < templateCenterX+templateWidth/8
                && rowIndex > templateCenterY-templateHeight/8
                && rowIndex < templateCenterY+templateHeight/8 )
              {
                CV_IMAGE_ELEM (objectMask, unsigned char, rowIndex, colIndex) = 255;
              }
              
            }
          }
 
          
          cvShowImage ("backgroundMask", backgroundMask);
          cvWaitKey (10);
          cvShowImage ("objectMask", objectMask);
          cvWaitKey (10);
          
          IplImage * refinedMask = segmentObject (colorImageClone, backgroundMask, objectMask, objectROI);
          
          // get discretized disparity data
          ::ftd::discretizers::MaskedDisparityDiscretizer discretizer;
          discretizer.discretize(
            colorImage,
            refinedMask,
            disparityMsg,
            regionWidth,
            regionHeight,
            maskedDisparityData );          
            
            
          cvReleaseImage (&refinedMask);
          
          cvReleaseImage (&objectMask);
          cvReleaseImage (&backgroundMask);
        }*/
        
        IplImage * maskedSmoothedImage = cvCreateImage (cvGetSize (smoothedImage), IPL_DEPTH_32F, 1);
        cvSetZero (maskedSmoothedImage);
        unsigned char * maskedDiscretizedData = new unsigned char[horizontalSamples*verticalSamples];
        {
          IplImage * resizedColorImageClone = cvCreateImage (cvSize (colorImageClone->width/2, colorImageClone->height/2), IPL_DEPTH_32F, 3);
          cvResize (colorImageClone, resizedColorImageClone); 
        
          const int templateWidth = templateDetector1_->getTemplateWidth ()/2;
          const int templateHeight = templateDetector1_->getTemplateHeight ()/2;
          
          const int templateCenterX = templateStartX/2 + templateWidth/2;
          const int templateCenterY = templateStartY/2 + templateHeight/2;
          
          CvRect objectROI = cvRect (templateStartX/2-templateWidth/2, templateStartY/2-templateHeight/2, 2*templateWidth, 2*templateHeight);
          
          IplImage * objectMask = cvCreateImage (cvGetSize (resizedColorImageClone), IPL_DEPTH_8U, 1);
          IplImage * backgroundMask = cvCreateImage (cvGetSize (resizedColorImageClone), IPL_DEPTH_8U, 1);
          cvSetZero (objectMask);
          cvSetZero (backgroundMask);
          
          for (int rowIndex = 0; rowIndex < backgroundMask->height; ++rowIndex)
          {
            for (int colIndex = 0; colIndex < backgroundMask->width; ++colIndex)
            {
              if ( colIndex > objectROI.x+templateWidth/4
                && colIndex < objectROI.x+objectROI.width-templateWidth/4
                && rowIndex > objectROI.y+templateHeight/4
                && rowIndex < objectROI.y+objectROI.height-templateHeight/4 )
              {
                CV_IMAGE_ELEM (backgroundMask, unsigned char, rowIndex, colIndex) = 255;
              }
              
              if ( colIndex > templateCenterX-templateWidth/8
                && colIndex < templateCenterX+templateWidth/8
                && rowIndex > templateCenterY-templateHeight/8
                && rowIndex < templateCenterY+templateHeight/8 )
              {
                CV_IMAGE_ELEM (objectMask, unsigned char, rowIndex, colIndex) = 255;
              }
              
            }
          }
 
          
          cvShowImage ("backgroundMask", backgroundMask);
          cvWaitKey (10);
          cvShowImage ("objectMask", objectMask);
          cvWaitKey (10);
          
          IplImage * refinedMask = segmentObject (resizedColorImageClone, backgroundMask, objectMask, objectROI);
          
          IplImage * resizedRefinedMask = cvCreateImage (cvGetSize (colorImageClone), IPL_DEPTH_8U, 1);
          cvResize (refinedMask, resizedRefinedMask);
          
          // get discretized disparity data
          ::ftd::discretizers::MaskedDisparityDiscretizer discretizer;
          discretizer.discretize(
            colorImage,
            resizedRefinedMask,
            disparityMsg,
            regionWidth,
            regionHeight,
            maskedDisparityData );          
            
            
            
          
          // mask smoothed image
          std::vector<unsigned char> maskedDiscretizedDataVec (horizontalSamples*verticalSamples);
          
          cvCopy (smoothedImage, maskedSmoothedImage, resizedRefinedMask);
          
          GradientDiscretizer::discretize(
            maskedSmoothedImage,
            regionWidth_,
            regionHeight_,
            maskedDiscretizedDataVec,
            8 );
        
          int elementIndex = 0;
          for (std::vector<unsigned char>::const_iterator iter = maskedDiscretizedDataVec.begin (); 
          iter != maskedDiscretizedDataVec.end (); 
          ++iter)
          {
            maskedDiscretizedData[elementIndex] = *iter;
            
            ++elementIndex;
          }    
          
            
            
            
          cvReleaseImage (&refinedMask);
          
          cvReleaseImage (&objectMask);
          cvReleaseImage (&backgroundMask);
          
          cvReleaseImage (&resizedRefinedMask);
          cvReleaseImage (&resizedColorImageClone);
        }        
        ::ros::Time t_segObj2 = ::ros::Time::now ();
        
        ROS_INFO ("object segmentation time: %f", (t_segObj2-t_segObj1).toSec ());


        /*CvRect initialROI = cvRect (templateStartX, templateStartY-templateDetector1_->getTemplateHeight ()/4, templateDetector1_->getTemplateWidth (), 1.5f*templateDetector1_->getTemplateHeight ());
        CvRect roi = estimateROI (smoothedImage, initialROI);
        
        CvPoint p1 = cvPoint (roi.x, roi.y);
        CvPoint p2 = cvPoint (roi.x + roi.width, roi.y + roi.height);

        cvRectangle (colorImage, p1, p2, CV_RGB (255, 128, 128), 3, 8, 0);*/              
    
        int maxResponse = 0;
        for (int rowIndex = 0; rowIndex < templateVerticalSamples; ++rowIndex)
        {
          for (int colIndex = 0; colIndex < templateHorizontalSamples; ++colIndex)
          {
            const int tmpColIndex = colIndex + templateStartX/regionWidth;
            const int tmpRowIndex = rowIndex + templateStartY/regionHeight;
            
            templateData[rowIndex*templateHorizontalSamples + colIndex] = maskedDiscretizedData[tmpRowIndex*horizontalSamples + tmpColIndex];
            templateDisparityData[rowIndex*templateHorizontalSamples + colIndex] = maskedDisparityData[tmpRowIndex*horizontalSamples + tmpColIndex];
            
            if (discretizedData[tmpRowIndex*horizontalSamples + tmpColIndex] != 0)
            {
              ++maxResponse;
            }
          }
        }
                
//        templateDetector1_->addNewTemplate (templateData, classIndex, templateHorizontalSamples*templateVerticalSamples, roi);
//        templateDetector1_->addNewTemplate (templateData, classIndex, maxResponse, roi);
        templateDetector1_->addNewTemplate (templateData, classIndex, maxResponse);
        templateDetector2_->addNewTemplate (templateDisparityData, classIndex, templateHorizontalSamples*templateVerticalSamples);
        
        cvSetImageROI (maskedSmoothedImage, cvRect (templateStartX, templateStartY, templateDetector1_->getTemplateWidth (), templateDetector1_->getTemplateHeight ()));
        templateDetector1_->addContour (maskedSmoothedImage);
        cvResetImageROI (maskedSmoothedImage);
        
        cvReleaseImage (&maskedSmoothedImage);
        
        delete[] templateData;
        delete[] templateDisparityData;
        
        ROS_INFO("Learned a new template from mouse click");	

        templateDetector1_->clearClusters ();
        templateDetector1_->clusterHeuristically (4);
        templateDetector2_->clearClusters ();
        templateDetector2_->clusterHeuristically (4);
        ROS_INFO("Created clusters");
        
        templateDetector1_->save (templateFileName1_);
        templateDetector2_->save (templateFileName2_);
        ROS_INFO("Templates saved");
        
        delete[] maskedDiscretizedData;
      }
		}

	  for (int classIndex = 0; classIndex < templateDetector1_->getNumOfClasses(); ++classIndex)
	  {
		  ::ftd::emptyPointerList(candidateList[classIndex]);
	  }
	}
	
	::ros::Time t6 = ::ros::Time::now ();
	
	if (objectDetected)
	{
	  cvScale (colorImage, colorImage, 1.0f/255.0f);
	  cvShowImage ("colorImage", colorImage);
    int pressedKey = cvWaitKey(10);

    if (pressedKey == 27)
    {
      stop_ = true;
    }
  }
  else
  {
    ::io::Mouse mouse_;
  
    cvNamedWindow ( "colorImage", CV_WINDOW_AUTOSIZE );
    mouse_.start("colorImage");
    
	  cvScale (colorImage, colorImage, 1.0f/255.0f);
	  cvShowImage ("colorImage", colorImage);
    int pressedKey = cvWaitKey(-1);

    if (pressedKey == 27)
    {
      stop_ = true;
    }

    // get current mouse position
    const int mousePosX = mouse_.getX ();
    const int mousePosY = mouse_.getY ();
    const int mouseEvent = mouse_.getEvent();
    
    std::cerr << "mouse: " << mousePosX << ", " << mousePosY << std::endl;
    
    
    // visualize template area
    int templateCenterX = -1;
    int templateCenterY = -1;
    if (mousePosX >= 0 && mousePosY >= 0)
    {
	    templateCenterX = mousePosX;
	    templateCenterY = mousePosY;		
    }
    if ( templateCenterX - templateDetector1_->getTemplateWidth()/2 >= 0 ||
	       templateCenterY - templateDetector1_->getTemplateHeight()/2 >= 0 ||
	       templateCenterX + templateDetector1_->getTemplateWidth()/2 <= grayImage->width-1 ||
	       templateCenterY + templateDetector1_->getTemplateHeight()/2 <= grayImage->height-1 )
    {
	    CvPoint pt1;
	    CvPoint pt2;
	    CvPoint pt3;
	    CvPoint pt4;

	    pt1.x = templateCenterX-templateDetector1_->getTemplateWidth()/2;
	    pt1.y = templateCenterY-templateDetector1_->getTemplateHeight()/2;
	    pt2.x = templateCenterX+templateDetector1_->getTemplateWidth()/2;
	    pt2.y = templateCenterY-templateDetector1_->getTemplateHeight()/2;
	    pt3.x = templateCenterX+templateDetector1_->getTemplateWidth()/2;
	    pt3.y = templateCenterY+templateDetector1_->getTemplateHeight()/2;
	    pt4.x = templateCenterX-templateDetector1_->getTemplateWidth()/2;
	    pt4.y = templateCenterY+templateDetector1_->getTemplateHeight()/2;

	    //if (showRectangle)
	    {
		    /*cvLine(resizedImage,pt1,pt2,CV_RGB(0,0,0),3);
		    cvLine(resizedImage,pt2,pt3,CV_RGB(0,0,0),3);
		    cvLine(resizedImage,pt3,pt4,CV_RGB(0,0,0),3);
		    cvLine(resizedImage,pt4,pt1,CV_RGB(0,0,0),3);

		    cvLine(resizedImage,pt1,pt2,CV_RGB(255,255,0),1);
		    cvLine(resizedImage,pt2,pt3,CV_RGB(255,255,0),1);
		    cvLine(resizedImage,pt3,pt4,CV_RGB(255,255,0),1);
		    cvLine(resizedImage,pt4,pt1,CV_RGB(255,255,0),1);*/
	    }
    }


    // create template if right mouse button has been pressed
    if (templateCenterX != -1 && templateCenterY != -1 /*&& mouseEvent == 2*/)
    {
      // correct template center based on disparity
      {
        const float maxDisparityDist = 7.5f;
      
        const float centerDisparity = CV_IMAGE_ELEM (disparityImage, float, templateCenterY, templateCenterX);
//        std::cerr << "centerDisparity: " << centerDisparity << std::endl;
        
        if (centerDisparity > 1.0f)
        {
          const int startX = templateCenterX-templateDetector1_->getTemplateWidth ()/2;
          const int startY = templateCenterY-templateDetector1_->getTemplateHeight ()/2;
          
          const int endX = startX+templateDetector1_->getTemplateWidth ();
          const int endY = startY+templateDetector1_->getTemplateHeight ();
          
          int newCenterX = 0.0f;
          int newCenterY = 0.0f;
          int counter = 0;
          for (int rowIndex = startY; rowIndex < endY; ++rowIndex)
          {
            for (int colIndex = startX; colIndex < endX; ++colIndex)
            {
              if (abs(CV_IMAGE_ELEM (disparityImage, float, rowIndex, colIndex) - centerDisparity) < maxDisparityDist)
              {
                newCenterX += colIndex;
                newCenterY += rowIndex;
                ++counter;
              }
              else
              {
                CV_IMAGE_ELEM (disparityImage, float, rowIndex, colIndex) = 0;
              }
            }
          }
          
//          std::cerr << "old: " << templateCenterX << ", " << templateCenterY << std::endl;

          if (counter > 0)
          {
            templateCenterX = newCenterX/counter;
            //templateCenterY = newCenterY/counter;
          }
          
//          std::cerr << "new: " << templateCenterX << ", " << templateCenterY << std::endl;
        }
        
//        cvScale (disparityImage, disparityImage, 1.0/255.0);
//        cvShowImage ("disp", disparityImage);
//        cvScale (disparityImage, disparityImage, 255.0);
//        cvWaitKey (10);
      }
      

          
      const int regionWidth = templateDetector1_->getSamplingSize ();
      const int regionHeight = templateDetector1_->getSamplingSize ();
      const int templateHorizontalSamples = templateDetector1_->getTemplateWidth () / templateDetector1_->getSamplingSize ();
      const int templateVerticalSamples = templateDetector1_->getTemplateHeight () / templateDetector1_->getSamplingSize ();
      unsigned char * templateData = new unsigned char[templateHorizontalSamples * templateVerticalSamples];
      unsigned char * templateDisparityData = new unsigned char[templateHorizontalSamples * templateVerticalSamples];
      
      
      //segment object
      std::vector<unsigned char> maskedDisparityData (horizontalSamples * verticalSamples, 0);
      IplImage * maskedSmoothedImage = cvCreateImage (cvGetSize (smoothedImage), IPL_DEPTH_32F, 1);
      cvSetZero (maskedSmoothedImage);
      unsigned char * maskedDiscretizedData = new unsigned char[horizontalSamples*verticalSamples];
      {
        const int templateWidth = templateDetector1_->getTemplateWidth ();
        const int templateHeight = templateDetector1_->getTemplateHeight ();
        
        const int templateStartX = templateCenterX-templateWidth/2;
        const int templateStartY = templateCenterY-templateHeight/2;
        
        CvRect objectROI = cvRect (templateStartX-templateWidth/2, templateStartY-templateHeight/2, 2*templateWidth, 2*templateHeight);
        
        IplImage * objectMask = cvCreateImage (cvGetSize (colorImage), IPL_DEPTH_8U, 1);
        IplImage * backgroundMask = cvCreateImage (cvGetSize (colorImage), IPL_DEPTH_8U, 1);
        cvSetZero (objectMask);
        cvSetZero (backgroundMask);
        
        for (int rowIndex = 0; rowIndex < backgroundMask->height; ++rowIndex)
        {
          for (int colIndex = 0; colIndex < backgroundMask->width; ++colIndex)
          {
            if ( colIndex > objectROI.x+templateWidth/4
              && colIndex < objectROI.x+objectROI.width-templateWidth/4
              && rowIndex > objectROI.y+templateHeight/4
              && rowIndex < objectROI.y+objectROI.height-templateHeight/4 )
            {
              CV_IMAGE_ELEM (backgroundMask, unsigned char, rowIndex, colIndex) = 255;
            }
            
            if ( colIndex > templateCenterX-templateWidth/8
              && colIndex < templateCenterX+templateWidth/8
              && rowIndex > templateCenterY-templateHeight/8
              && rowIndex < templateCenterY+templateHeight/8 )
            {
              CV_IMAGE_ELEM (objectMask, unsigned char, rowIndex, colIndex) = 255;
            }
            
          }
        }
        
        cvShowImage ("backgroundMask", backgroundMask);
        cvWaitKey (10);
        cvShowImage ("objectMask", objectMask);
        cvWaitKey (10);

        
        IplImage * refinedMask = segmentObject (colorImageClone, backgroundMask, objectMask, objectROI);
        
        // get discretized disparity data
        ::ftd::discretizers::MaskedDisparityDiscretizer discretizer;
        discretizer.discretize(
          colorImage,
          refinedMask,
          disparityMsg,
          regionWidth,
          regionHeight,
          maskedDisparityData );          
          
          
          
        // mask smoothed image
        std::vector<unsigned char> maskedDiscretizedDataVec (horizontalSamples*verticalSamples);
        
        cvCopy (smoothedImage, maskedSmoothedImage, refinedMask);
        
        GradientDiscretizer::discretize(
          maskedSmoothedImage,
          regionWidth_,
          regionHeight_,
          maskedDiscretizedDataVec,
          8 );
      
        int elementIndex = 0;
        for (std::vector<unsigned char>::const_iterator iter = maskedDiscretizedDataVec.begin (); 
        iter != maskedDiscretizedDataVec.end (); 
        ++iter)
        {
          maskedDiscretizedData[elementIndex] = *iter;
          
          ++elementIndex;
        }              
          
          
        cvReleaseImage (&refinedMask);
        
        cvReleaseImage (&objectMask);
        cvReleaseImage (&backgroundMask);
      }
        
              
      int maxResponse = 0;
      for (int rowIndex = 0; rowIndex < templateVerticalSamples; ++rowIndex)
      {
        for (int colIndex = 0; colIndex < templateHorizontalSamples; ++colIndex)
        {
          const int tmpColIndex = colIndex + (templateCenterX-templateDetector1_->getTemplateWidth ()/2)/regionWidth;
          const int tmpRowIndex = rowIndex + (templateCenterY-templateDetector1_->getTemplateHeight ()/2)/regionHeight;
          
          templateData[rowIndex*templateHorizontalSamples + colIndex] = maskedDiscretizedData[tmpRowIndex*horizontalSamples + tmpColIndex];
          templateDisparityData[rowIndex*templateHorizontalSamples + colIndex] = maskedDisparityData[tmpRowIndex*horizontalSamples + tmpColIndex];
          
          if (discretizedData[tmpRowIndex*horizontalSamples + tmpColIndex] != 0)
          {
            ++maxResponse;
          }
        }
      }
              
//	    templateDetector1_->addNewTemplate (templateData, 0, templateHorizontalSamples*templateVerticalSamples, roi);
//	    templateDetector1_->addNewTemplate (templateData, 0, maxResponse, roi);
	    templateDetector1_->addNewTemplate (templateData, 0, maxResponse);
	    templateDetector2_->addNewTemplate (templateDisparityData, 0, templateHorizontalSamples*templateVerticalSamples);
	    
      const int templateWidth = templateDetector1_->getTemplateWidth ();
      const int templateHeight = templateDetector1_->getTemplateHeight ();
      
      const int templateStartX = templateCenterX-templateWidth/2;
      const int templateStartY = templateCenterY-templateHeight/2;
      
      cvSetImageROI (maskedSmoothedImage, cvRect (templateStartX, templateStartY, templateDetector1_->getTemplateWidth (), templateDetector1_->getTemplateHeight ()));
      templateDetector1_->addContour (maskedSmoothedImage);
      cvResetImageROI (maskedSmoothedImage);
      
      cvReleaseImage (&maskedSmoothedImage);
	    
	    delete[] templateData;
	    delete[] templateDisparityData;
	    delete[] maskedDiscretizedData;
	    
	    ROS_INFO("Learned a new template from mouse click");	
	
	    templateDetector1_->clearClusters ();
	    templateDetector1_->clusterHeuristically (4);
	    templateDetector2_->clearClusters ();
	    templateDetector2_->clusterHeuristically (4);
	    ROS_INFO("Created clusters");

      templateDetector1_->save (templateFileName1_);
      templateDetector2_->save (templateFileName2_);
      ROS_INFO("Templates saved");
    }    
  }

  ::ros::Time t7 = ::ros::Time::now ();
  
  ROS_INFO ("timings: %f, %f, %f, %f, %f, %f, %f",
    (t2-t1).toSec (),
    (t3-t2).toSec (),
    (t4-t3).toSec (),
    (t5-t4).toSec (),
    (t6-t5).toSec (),
    (t7-t6).toSec () );
    
  
  cvReleaseImage (&colorImage);
  cvReleaseImage (&colorImageClone);
  cvReleaseImage (&grayImage);
  cvReleaseImage (&smoothedImage);
  cvReleaseImage (&disparityImage);
  
  delete[] discretizedData;
  
  processed_ = true;    
}



// ----------------------------------------------------------------------------
void
ftd::learners::
BimodalAdaptiveTemplateLearnerBAG::
computeROIsFromPointCloud (
  const ::sensor_msgs::ImageConstPtr & imageMsg,
  const ::sensor_msgs::CameraInfoConstPtr & camInfoMsg,
  const ::sensor_msgs::PointCloud2ConstPtr & cloudMsg,
  dot::PerceptionData & perceptionData )
{
  ::sensor_msgs::Image image_rect;
  pcl::PointCloud<pcl::PointXYZ> cloud_rgb;
//  PointCloud2ImageColor::getColorPointCloud2 (cloudMsg, imageMsg, image_rect, cloud_rgb);

  point_cloud::fromMsg (*cloudMsg, cloud_rgb);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_rgb_ptr = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> > (cloud_rgb);
//  ImageConstPtr image_rect_ptr = boost::make_shared<Image> (image_rect);
  ROS_INFO ("Got a PointCloud<PointXYZRGB> with %d points.", (int)cloud_rgb.points.size ());
  
  ftd::ExtractObjectRegions eor;
  eor.setInputCloud (cloud_rgb_ptr);
  eor.setInputImage (imageMsg);
  eor.setInputCameraInfo (camInfoMsg);
  eor.extract (perceptionData, true);
}


// ----------------------------------------------------------------------------
IplImage *
ftd::learners::
BimodalAdaptiveTemplateLearnerBAG::
getColorImage32F ( 
  IplImage * image )
{
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
  
  return colorImage;
}


// ----------------------------------------------------------------------------
IplImage *
ftd::learners::
BimodalAdaptiveTemplateLearnerBAG::
getGrayImage32F ( 
  IplImage * image )
{
  IplImage * grayImage = NULL;
  
  if (image->depth == 32)
  {
    grayImage = cvCreateImage(cvGetSize(image), IPL_DEPTH_32F, 1);
    cvCvtColor(image, grayImage, CV_BGR2GRAY);
  }
  else if (image->depth == 8)
  {
    if (image->nChannels == 3)
    {
      IplImage * tmpGrayImage = cvCreateImage(cvGetSize(image), IPL_DEPTH_8U, 1);
      cvCvtColor(image, tmpGrayImage, CV_BGR2GRAY);
      
      grayImage = cvCreateImage(cvGetSize(image), IPL_DEPTH_32F, 1);
      cvConvert(tmpGrayImage, grayImage);
      
      cvReleaseImage(&tmpGrayImage);
    }
    else if (image->nChannels == 1)
    {
      grayImage = cvCreateImage(cvGetSize(image), IPL_DEPTH_32F, 1);
      cvConvert(image, grayImage);
    }
    else
    {
      ROS_INFO("Number of image channels is not supported!");
      return NULL;
    }
  }
  else
  {
    ROS_INFO("Image depth not supported!");
    return NULL;
  }
  
  return grayImage;
}


// ----------------------------------------------------------------------------
IplImage *
ftd::learners::
BimodalAdaptiveTemplateLearnerBAG::
getDisparityImage ( 
  const ::stereo_msgs::DisparityImageConstPtr & disparityMsg )
{
  CvMat * disparityMatrix = cvCreateMatHeader (disparityMsg->image.height, disparityMsg->image.width, CV_32FC1);
  cvSetData (disparityMatrix, static_cast<void*> (const_cast<unsigned char*> (&(disparityMsg->image.data[0]))), disparityMsg->image.step);
  
  IplImage * disparityImage = cvCreateImage (cvSize (disparityMatrix->cols, disparityMatrix->rows), IPL_DEPTH_32F, 1);
  cvSetZero (disparityImage);


  for (int rowIndex = 0; rowIndex < disparityImage->height; ++rowIndex)
  {
    for (int colIndex = 0; colIndex < disparityImage->width; ++colIndex)
    {
      const float dispValue = CV_MAT_ELEM (*disparityMatrix, float, rowIndex, colIndex);
      if (dispValue > 50.0f /*&& dispValue < 120.0f*/)
      {
        CV_IMAGE_ELEM (disparityImage, float, rowIndex, colIndex) = dispValue;
      }
    }
  }
  
  return disparityImage;
}


// ----------------------------------------------------------------------------
void
ftd::learners::
BimodalAdaptiveTemplateLearnerBAG::
drawTemplateBorder ( 
  IplImage * image,
  CvScalar color,
  const int templateStartX,
  const int templateStartY,
  const int templateWidth,
  const int templateHeight )
{
  cvLine (
    image,
    cvPoint (templateStartX, templateStartY),
    cvPoint (templateStartX+templateWidth, templateStartY),
    color,
    2 );
  cvLine (
    image,
    cvPoint (templateStartX+templateWidth, templateStartY),
    cvPoint (templateStartX+templateWidth, templateStartY+templateHeight),
    color,
    2 );
  cvLine (
    image,
    cvPoint (templateStartX+templateWidth, templateStartY+templateHeight),
    cvPoint (templateStartX, templateStartY+templateHeight),
    color,
    2 );
  cvLine (
    image,
    cvPoint (templateStartX, templateStartY+templateHeight),
    cvPoint (templateStartX, templateStartY),
    color,
    2 );      
}   


// ----------------------------------------------------------------------------
bool
ftd::learners::
BimodalAdaptiveTemplateLearnerBAG::
addTemplate (
  const int templateCenterX,
  const int templateCenterY,
  IplImage * smoothedImage,
  IplImage * objectMask,
  unsigned char * discretizedData,
  const int horizontalSamples,
  const int verticalSamples )
{
            
  const int regionWidth = templateDetector1_->getSamplingSize ();
  const int regionHeight = templateDetector1_->getSamplingSize ();
  const int templateHorizontalSamples = templateDetector1_->getTemplateWidth () / templateDetector1_->getSamplingSize ();
  const int templateVerticalSamples = templateDetector1_->getTemplateHeight () / templateDetector1_->getSamplingSize ();
  unsigned char * templateData = new unsigned char[templateHorizontalSamples * templateVerticalSamples];

  
  //segment object
  ROS_INFO("apply mask");
  IplImage * maskedSmoothedImage = cvCreateImage (cvGetSize (smoothedImage), IPL_DEPTH_32F, 1);
  cvSetZero (maskedSmoothedImage);
  cvCopy (smoothedImage, maskedSmoothedImage, objectMask);
  
#ifdef USE_RESTRICTED_NUM_OF_GRADIENTS
  std::vector<unsigned char> discretizedData2(horizontalSamples*verticalSamples, 0);
  std::vector<float> strength(horizontalSamples*verticalSamples, 0.0f);
  ::ftd::discretizers::GradientDiscretizer discretizer;
  discretizer.discretize (
    smoothedImage,
    objectMask,
    regionWidth,
    regionHeight,
    discretizedData2,
    strength,
    8 );  

  float * strengthData = new float[templateHorizontalSamples * templateVerticalSamples];

  ROS_INFO("copy template data");
  int maxResponse = 0;
  for (int rowIndex = 0; rowIndex < templateVerticalSamples; ++rowIndex)
  {
    for (int colIndex = 0; colIndex < templateHorizontalSamples; ++colIndex)
    {
      const int tmpColIndex = colIndex + (templateCenterX-templateDetector1_->getTemplateWidth ()/2)/regionWidth;
      const int tmpRowIndex = rowIndex + (templateCenterY-templateDetector1_->getTemplateHeight ()/2)/regionHeight;
      
      templateData[rowIndex*templateHorizontalSamples + colIndex] = discretizedData2[tmpRowIndex*horizontalSamples + tmpColIndex];
      strengthData[rowIndex*templateHorizontalSamples + colIndex] = strength[tmpRowIndex*horizontalSamples + tmpColIndex];
      
      if (discretizedData2[tmpRowIndex*horizontalSamples + tmpColIndex] != 0)
      {
        ++maxResponse;
      }
    }
  }
  
  const int numOfDiscretizedGradients = 120;
  for (int binCounter = 0; binCounter < ((templateHorizontalSamples*templateVerticalSamples)-numOfDiscretizedGradients); ++binCounter)
  {
    int minIndex = 0;
    float minValue = 10e100;
    for (int binIndex = 0; binIndex < (templateHorizontalSamples*templateVerticalSamples); ++binIndex)
    {
      if (strengthData[binIndex] < minValue)
      {
        minValue = strengthData[binIndex];
        minIndex = binIndex;
      }
    }
    
    strengthData[minIndex] = 10e100;
    templateData[minIndex] = 0;
  }  
  
  maxResponse = 0;
  for (int index = 0; index < (templateHorizontalSamples * templateVerticalSamples); ++index)
  {
    if (templateData[index] != 0)
    {
      ++maxResponse;
    }
  }
  
  delete[] strengthData;
#else          
  ROS_INFO("copy template data");
  int maxResponse = 0;
  for (int rowIndex = 0; rowIndex < templateVerticalSamples; ++rowIndex)
  {
    for (int colIndex = 0; colIndex < templateHorizontalSamples; ++colIndex)
    {
      const int tmpColIndex = colIndex + (templateCenterX-templateDetector1_->getTemplateWidth ()/2)/regionWidth;
      const int tmpRowIndex = rowIndex + (templateCenterY-templateDetector1_->getTemplateHeight ()/2)/regionHeight;
      
      templateData[rowIndex*templateHorizontalSamples + colIndex] = discretizedData[tmpRowIndex*horizontalSamples + tmpColIndex];
      
      if (discretizedData[tmpRowIndex*horizontalSamples + tmpColIndex] != 0)
      {
        ++maxResponse;
      }
    }
  }
#endif
  
  std::cerr << "maxResponse: " << maxResponse << std::endl;
          
  ROS_INFO("add template");
  
  bool templateAdded = false;
#ifdef USE_GRADIENTS
#ifdef USE_RESTRICTED_NUM_OF_GRADIENTS
  if (maxResponse >= 100)
#else
  if (maxResponse >= 150)
#endif
#endif
#ifdef USE_SURFACE_NORMALS
  if (maxResponse >= 50)
#endif
  {
    const int templateWidth = templateDetector1_->getTemplateWidth ();
    const int templateHeight = templateDetector1_->getTemplateHeight ();
    
    const int templateStartX = templateCenterX-templateWidth/2;
    const int templateStartY = templateCenterY-templateHeight/2;
    
    templateDetector1_->addNewTemplate (templateData, 0, maxResponse, cvRect (templateStartX, templateStartY, templateWidth, templateHeight));
  
    ROS_INFO("add contour");
    cvSetImageROI (maskedSmoothedImage, cvRect (templateStartX, templateStartY, templateDetector1_->getTemplateWidth (), templateDetector1_->getTemplateHeight ()));
    templateDetector1_->addContour (maskedSmoothedImage);
    cvResetImageROI (maskedSmoothedImage);
    
    templateAdded = true;
  }
    
  cvReleaseImage (&maskedSmoothedImage);
  
  delete[] templateData;
  
  templateDetector1_->clearClusters ();
  templateDetector1_->clusterHeuristically (4);
  ROS_INFO("Created clusters");

  //templateDetector1_->save (templateFileName1_);
  
  //saveTemplates (templateFileName1_, templateFileName2_);
  //ROS_INFO("Templates saved");
  
  return templateAdded;
}


// ----------------------------------------------------------------------------
void
ftd::learners::
BimodalAdaptiveTemplateLearnerBAG::
addValidationData (
  IplImage * smoothedImage,
  IplImage * disparityImage,
  IplImage * mask,
  const int objectCenterX,
  const int objectCenterY,
  sensor_msgs::PointCloud2 & cloud )
{
  const int templateWidth = templateDetector1_->getTemplateWidth ();
  const int templateHeight = templateDetector1_->getTemplateHeight ();
  
  const int templateStartX = objectCenterX - templateWidth/2;
  const int templateStartY = objectCenterY - templateHeight/2;

  // create gradient templates
  IplImage * sobelDx = cvCreateImage (cvSize (templateWidth, templateHeight), IPL_DEPTH_32F, 1);
  IplImage * sobelDy = cvCreateImage (cvSize (templateWidth, templateHeight), IPL_DEPTH_32F, 1);
  IplImage * sobelMagnitude = cvCreateImage (cvSize (templateWidth, templateHeight), IPL_DEPTH_32F, 1);
  //IplImage * sobelAngle = cvCreateImage (cvSize (templateWidth, templateHeight), IPL_DEPTH_32F, 1);
  
  CvRect roi = cvRect (templateStartX, templateStartY, templateWidth, templateHeight);
  
  cvSetImageROI (smoothedImage, roi);
  cvSetImageROI (disparityImage, roi);
  cvSetImageROI (mask, roi);

  cvSobel (smoothedImage, sobelDx, 1, 0, 3);
  cvSobel (smoothedImage, sobelDy, 0, 1, 3);

  //cvCartToPolar (sobelDx, sobelDy, sobelMagnitude, sobelAngle, 1); // the "1" means that the angles are in degree
  cvCartToPolar (sobelDx, sobelDy, sobelMagnitude);
  
  for (int rowIndex = 0; rowIndex < sobelDx->height; ++rowIndex)
  {
    for (int colIndex = 0; colIndex < sobelDx->width; ++colIndex)
    {
      if (CV_IMAGE_ELEM (mask, unsigned char, rowIndex+templateStartY, colIndex+templateStartX) == 0)
      {
        CV_IMAGE_ELEM (sobelDx, float, rowIndex, colIndex) = 0.0f;
        CV_IMAGE_ELEM (sobelDy, float, rowIndex, colIndex) = 0.0f;
        CV_IMAGE_ELEM (sobelMagnitude, float, rowIndex, colIndex) = 0.0f;
      }
    }
  }
  
  // create disparity template
  IplImage * disparityTemplate = cvCreateImage (cvSize (templateWidth, templateHeight), IPL_DEPTH_32F, 1);
  cvSetZero (disparityTemplate);
  
  cvCopy (disparityImage, disparityTemplate, mask);
  
  cvResetImageROI (smoothedImage);
  cvResetImageROI (disparityImage);
  cvResetImageROI (mask);
  
  //cvReleaseImage (&sobelAngle);
  //cvReleaseImage (&sobelMagnitude);
  
  IplImage * rSobelDx = cvCreateImage (cvSize (sobelDx->width/2, sobelDx->height/2), IPL_DEPTH_32F, 1);
  IplImage * rSobelDy = cvCreateImage (cvSize (sobelDx->width/2, sobelDx->height/2), IPL_DEPTH_32F, 1);
  IplImage * rSobelMagnitude = cvCreateImage (cvSize (sobelDx->width/2, sobelDx->height/2), IPL_DEPTH_32F, 1);
  IplImage * rDisparityTemplate = cvCreateImage (cvSize (sobelDx->width/2, sobelDx->height/2), IPL_DEPTH_32F, 1);
  
  cvResize (sobelDx, rSobelDx);
  cvResize (sobelDy, rSobelDy);
  cvResize (sobelMagnitude, rSobelMagnitude);
  cvResize (disparityTemplate, rDisparityTemplate);
  
  sobelDxTemplates_.push_back (rSobelDx);
  sobelDyTemplates_.push_back (rSobelDy);
  sobelMagnitudeTemplates_.push_back (rSobelMagnitude);
  disparityTemplates_.push_back (rDisparityTemplate);
  
  pointCloudTemplates_.push_back (cloud);
  
  cvReleaseImage (&sobelDx);
  cvReleaseImage (&sobelDy);
  cvReleaseImage (&sobelMagnitude);
  cvReleaseImage (&disparityTemplate);
}


// ----------------------------------------------------------------------------
int
ftd::learners::
BimodalAdaptiveTemplateLearnerBAG::
validate (
  const int templateId,
  IplImage * smoothedImage,
  IplImage * disparityImage,
  const int templateCenterX,
  const int templateCenterY )
{
  // create gradient templates
  IplImage * sobelDx = cvCreateImage (cvGetSize (smoothedImage), IPL_DEPTH_32F, 1);
  IplImage * sobelDy = cvCreateImage (cvGetSize (smoothedImage), IPL_DEPTH_32F, 1);
  IplImage * sobelMagnitude = cvCreateImage (cvGetSize (smoothedImage), IPL_DEPTH_32F, 1);
  
  cvSobel (smoothedImage, sobelDx, 1, 0, 3);
  cvSobel (smoothedImage, sobelDy, 0, 1, 3);

  cvCartToPolar (sobelDx, sobelDy, sobelMagnitude);
  
  const int templateWidth = templateDetector1_->getTemplateWidth ();
  const int templateHeight = templateDetector1_->getTemplateHeight ();
  
  const int templateStartX = templateCenterX - templateWidth/2;
  const int templateStartY = templateCenterY - templateHeight/2;
  
  IplImage * rSobelDxTemplate = sobelDxTemplates_[templateId];
  IplImage * rSobelDyTemplate = sobelDyTemplates_[templateId];
  IplImage * rSobelMagnitudeTemplate = sobelMagnitudeTemplates_[templateId];
  IplImage * rDisparityTemplate = disparityTemplates_[templateId];
  
  IplImage * sobelDxTemplate = cvCreateImage (cvSize (templateWidth, templateHeight), IPL_DEPTH_32F, 1);
  IplImage * sobelDyTemplate = cvCreateImage (cvSize (templateWidth, templateHeight), IPL_DEPTH_32F, 1);
  IplImage * sobelMagnitudeTemplate = cvCreateImage (cvSize (templateWidth, templateHeight), IPL_DEPTH_32F, 1);
  IplImage * disparityTemplate = cvCreateImage (cvSize (templateWidth, templateHeight), IPL_DEPTH_32F, 1);
  
  cvResize (rSobelDxTemplate, sobelDxTemplate);
  cvResize (rSobelDyTemplate, sobelDyTemplate);
  cvResize (rSobelMagnitudeTemplate, sobelMagnitudeTemplate);
  cvResize (rDisparityTemplate, disparityTemplate);
  
  
  cvShowImage ("disparityImage", disparityImage);
  cvWaitKey (10);
  cvShowImage ("disparityTemplate", disparityTemplate);
  cvWaitKey (10);
  cvShowImage ("sobelMagnitude", sobelMagnitude);
  cvWaitKey (10);

  cvShowImage ("sobelDxTemplate", sobelDxTemplate);
  cvWaitKey (10);
  cvShowImage ("sobelDyTemplate", sobelDyTemplate);
  cvWaitKey (10);
  cvShowImage ("sobelMagnitudeTemplate", sobelMagnitudeTemplate);
  cvWaitKey (10);
  cvShowImage ("sobelMagnitude", sobelMagnitude);
  cvWaitKey (10);
  
  
  int bestYOffset = 0;
  int bestXOffset = 0;
  float bestResponse = 0.0f;
  float bestDotResponse = 0.0f;
  float bestDispResponse = 0.0f;
  float bestRatio = 0.0f;
  for (int yOffset = -7; yOffset <= 7; ++yOffset)
  {
    for (int xOffset = -7; xOffset <= 7; ++xOffset)
    {
      
      int maxMagnitudeResponse = 0;
      int magnitudeResponse = 0;
      float dotResponse = 0.0f;
      float disparityResponse = 0.0f;
      int disparityCounter1 = 0;
      int disparityCounter2 = 0;
      for (int rowIndex = 0; rowIndex < disparityTemplate->height; ++rowIndex)
      {
        for (int colIndex = 0; colIndex < disparityTemplate->width; ++colIndex)
        {
          const int tmpRowIndex = templateStartY + rowIndex + yOffset;
          const int tmpColIndex = templateStartX + colIndex + xOffset;
          
          const float length1 = cvSqrt(
            CV_IMAGE_ELEM (sobelDxTemplate, float, rowIndex, colIndex)*CV_IMAGE_ELEM (sobelDxTemplate, float, rowIndex, colIndex)
            + CV_IMAGE_ELEM (sobelDyTemplate, float, rowIndex, colIndex)*CV_IMAGE_ELEM (sobelDyTemplate, float, rowIndex, colIndex) );
          const float length2 = cvSqrt(
            CV_IMAGE_ELEM (sobelDx, float, tmpRowIndex, tmpColIndex)*CV_IMAGE_ELEM (sobelDx, float, tmpRowIndex, tmpColIndex)
            + CV_IMAGE_ELEM (sobelDy, float, tmpRowIndex, tmpColIndex)*CV_IMAGE_ELEM (sobelDy, float, tmpRowIndex, tmpColIndex) );
                
//            if (CV_IMAGE_ELEM (sobelMagnitudeTemplate, float, rowIndex, colIndex) > 10.0f)
          if (length1 > 10.0f)
          {
            ++maxMagnitudeResponse;
//              if (CV_IMAGE_ELEM (sobelMagnitude, float, tmpRowIndex, tmpColIndex) > 10.0f)
            if (length2 > 10.0f)
            {
              ++magnitudeResponse;
              

                
              if (length1 < 1.0f || length2 < 1.0f) std::cerr << "===============================================================" << std::endl;

              
              //dotResponse += fabs (CV_IMAGE_ELEM (sobelDxTemplate, float, rowIndex, colIndex) * CV_IMAGE_ELEM (sobelDx, float, tmpRowIndex, tmpColIndex)
              //  / (length1*length2));
              //dotResponse += fabs (CV_IMAGE_ELEM (sobelDyTemplate, float, rowIndex, colIndex) * CV_IMAGE_ELEM (sobelDy, float, tmpRowIndex, tmpColIndex)
              //  / (length1*length2));

              dotResponse += fabs (CV_IMAGE_ELEM (sobelDxTemplate, float, rowIndex, colIndex) * CV_IMAGE_ELEM (sobelDx, float, tmpRowIndex, tmpColIndex)
                / (length1*length2)
                + CV_IMAGE_ELEM (sobelDyTemplate, float, rowIndex, colIndex) * CV_IMAGE_ELEM (sobelDy, float, tmpRowIndex, tmpColIndex)
                / (length1*length2));
            }
          }
          
          if (CV_IMAGE_ELEM (disparityTemplate, float, rowIndex, colIndex) > 0.0f)
          {
            ++disparityCounter1;
            if (CV_IMAGE_ELEM (disparityImage, float, tmpRowIndex, tmpColIndex) > 0.0f)
            {
              ++disparityCounter2;
              
              disparityResponse += fabs (CV_IMAGE_ELEM (disparityTemplate, float, rowIndex, colIndex) - CV_IMAGE_ELEM (disparityImage, float, tmpRowIndex, tmpColIndex));
            }
          }
          
        }
      }
      
      dotResponse /= static_cast<float> (magnitudeResponse+1.0f);
      disparityResponse /= static_cast<float> (disparityCounter2);
      disparityResponse = 1.0f/(disparityResponse+1.0f);
      
      bestDotResponse = std::max (bestDotResponse, dotResponse);
      bestDispResponse = std::max (bestDispResponse, disparityResponse);
      
      float combinedResponse = dotResponse * disparityResponse;
      
      if (combinedResponse > bestResponse)
      {
        bestResponse = combinedResponse;
        bestXOffset = xOffset;
        bestYOffset = yOffset;
      }

      
      if (maxMagnitudeResponse != 0)
      {
        bestRatio = std::max (bestRatio, static_cast<float> (magnitudeResponse) / static_cast<float> (maxMagnitudeResponse));
      }
      
      //if (disparityCounter2 != 0)
      //{
      //  std::cerr << xOffset << "/" << yOffset << ": " << disparityResponse/static_cast<float> (disparityCounter2) << std::endl;
      //}
      
      //if (maxMagnitudeResponse != 0)
      //{
      //  std::cerr << xOffset << "/" << yOffset << ": " << magnitudeResponse << "/" << maxMagnitudeResponse << " = " << (100*magnitudeResponse)/maxMagnitudeResponse << ", " << dotResponse/static_cast<float> (magnitudeResponse+1.0f) << std::endl;
      //}
      //else
      //{
      //  std::cerr << xOffset << "/" << yOffset << ": " << magnitudeResponse << "/" << maxMagnitudeResponse << std::endl;
      //}
      
    }
  }
  
  ROS_INFO ("best validation response: %f, %f, %f, %f", bestResponse, bestDotResponse, bestDispResponse, bestRatio);
  
  
  cvReleaseImage (&sobelDx);
  cvReleaseImage (&sobelDy);
  cvReleaseImage (&sobelMagnitude);
  
  cvReleaseImage (&sobelDxTemplate);  
  cvReleaseImage (&sobelDyTemplate);  
  cvReleaseImage (&sobelMagnitudeTemplate);  
  cvReleaseImage (&disparityTemplate);  
  
  return static_cast<int>(bestResponse*100.0f);
}


// ----------------------------------------------------------------------------
void
ftd::learners::
BimodalAdaptiveTemplateLearnerBAG::
callback2 (
  const ::sensor_msgs::ImageConstPtr & imageMsg,
  const ::sensor_msgs::CameraInfoConstPtr & camInfoMsg,
  const ::sensor_msgs::PointCloud2ConstPtr & cloudMsg,
  const ::stereo_msgs::DisparityImageConstPtr & disparityMsg )
{
  using ::ftd::BumpHunting;
  using ::ftd::discretizers::GradientDiscretizer;
  using ::ftd::discretizers::SurfaceNormalDiscretizer;
  using ::cv::Mat;
  
  
  std::cerr << "callback called" << std::endl;
 
  // get OpenCV image
  IplImage * image = bridge_.imgMsgToCv(imageMsg); 


  // create color image
  ROS_INFO("create color image");	
  IplImage * colorImage = getColorImage32F (image);
  IplImage * colorImageClone = static_cast<IplImage*>(cvClone(colorImage));
  
  
  // convert to gray value image
  ROS_INFO("create gray image ");	
  IplImage * grayImage = getGrayImage32F (image);
  
  
  // create smoothed image for gradient computation
  ROS_INFO("create smoothed image");	
  IplImage * smoothedImage = cvCreateImage (cvGetSize (grayImage), IPL_DEPTH_32F, 1);
  cvSmooth (grayImage, smoothedImage, CV_GAUSSIAN, 5);
  
  
  // compute ROIs
  ROS_INFO("compute ROIs");	
  ::ros::Time t1 = ::ros::Time::now ();
  dot::PerceptionData perceptionData;
  computeROIsFromPointCloud (imageMsg, camInfoMsg, cloudMsg, perceptionData);

  for (size_t i = 0; i < perceptionData.roi.size (); ++i)
  {
    CvPoint p1 = cvPoint (perceptionData.roi[i].x_offset, perceptionData.roi[i].y_offset);
    CvPoint p2 = cvPoint (perceptionData.roi[i].x_offset + perceptionData.roi[i].width, perceptionData.roi[i].y_offset + perceptionData.roi[i].height);

    cvRectangle (colorImage, p1, p2, CV_RGB (255, 0, 0), 3, 8, 0);
  }
  ::ros::Time t2 = ::ros::Time::now ();
  
  
  
  // use ROIs for initialization of the learning; if no ROI is found then skip this image
  ROS_INFO("found %d ROIs", perceptionData.roi.size ());	
  if (perceptionData.roi.size () >= 1)
  {
    
    // get disparity image
    ROS_INFO("get disparity image");	
    IplImage * disparityImage = getDisparityImage (disparityMsg);
    
    
    // refine segmentation using grabcut
    ROS_INFO("segment using grabcut");	
    std::cerr << perceptionData.roi[0].x_offset-50 << ", " <<
        perceptionData.roi[0].y_offset-50 << ", " <<
        perceptionData.roi[0].width+100 << ", " <<
        perceptionData.roi[0].height+100 << std::endl;
    IplImage * objectMask = cvCreateImage (cvGetSize (smoothedImage), IPL_DEPTH_8U, 1);
    int objectCenterX = 0;
    int objectCenterY = 0;
    bool skipImage = false;
    /*{
      ImageConstPtr maskedImageConstPtr = boost::make_shared<Image> (perceptionData.image_proc);
      IplImage * maskedImage = bridge_.imgMsgToCv(maskedImageConstPtr);
      IplImage * maskedImage8U = cvCreateImage (cvGetSize (maskedImage), IPL_DEPTH_8U, 3);
      IplImage * maskedImage8UG = cvCreateImage (cvGetSize (maskedImage), IPL_DEPTH_8U, 1);
      cvConvert (maskedImage, maskedImage8U);
      cvCvtColor (maskedImage8U, maskedImage8UG, CV_BGR2GRAY);
      
    ROS_INFO("segment using grabcut1");	
      IplImage * dilatedMask = cvCreateImage (cvGetSize (maskedImage8UG), IPL_DEPTH_8U, 1);
      IplImage * erodedMask = cvCreateImage (cvGetSize (maskedImage8UG), IPL_DEPTH_8U, 1);
    
    ROS_INFO("segment using grabcut2");	
      cvDilate (maskedImage8UG, dilatedMask, NULL, 10);
      cvErode (maskedImage8UG, erodedMask, NULL, 5);
      
    ROS_INFO("segment using grabcut3");	
      CvRect objectROI = cvRect (
        perceptionData.roi[0].x_offset-50,
        perceptionData.roi[0].y_offset-50,
        perceptionData.roi[0].width+100,
        perceptionData.roi[0].height+100 );
      
    ROS_INFO("segment using grabcut4");	
      IplImage * refinedMask = segmentObject (colorImageClone, dilatedMask, erodedMask, objectROI);
      
      cvDilate (refinedMask, objectMask, NULL, 5);
      
      
    ROS_INFO("segment using grabcut5");	
      // compute center of mass of refined mask
      int objectCenterCounter = 0;
      
      for (int rowIndex = 0; rowIndex < refinedMask->height; ++rowIndex)
      {
        for (int colIndex = 0; colIndex < refinedMask->width; ++colIndex)
        {
          if (CV_IMAGE_ELEM (refinedMask, unsigned char, rowIndex, colIndex) != 0)
          {
            objectCenterX += colIndex;
            objectCenterY += rowIndex;
            ++objectCenterCounter;
          }
        }
      }
      if (objectCenterCounter > 0)
      {
        objectCenterX /= objectCenterCounter;
        objectCenterY /= objectCenterCounter;
      }
      
      std::cerr << objectCenterX << ", " << objectCenterY << std::endl;
      
      const int templateWidth = templateDetector1_->getTemplateWidth ();
      const int templateHeight = templateDetector1_->getTemplateHeight ();
      
      //if (objectCenterX-templateWidth/2 < 0) objectCenterX = templateWidth/2+1;
      //if (objectCenterY-templateHeight/2 < 0) objectCenterY = templateHeight/2+1;
      //if (objectCenterX+templateWidth/2 >= smoothedImage->width) objectCenterX = smoothedImage->width - (templateWidth/2+1);
      //if (objectCenterY+templateHeight/2 >= smoothedImage->height) objectCenterY = smoothedImage->height - (templateHeight/2+1);

      if (objectCenterX-templateWidth/2 < 0 
        || objectCenterY-templateHeight/2 < 0 
        || objectCenterX+templateWidth/2 >= smoothedImage->width 
        || objectCenterY+templateHeight/2 >= smoothedImage->height)
      {
        skipImage = true;
      }

      std::cerr << objectCenterX << ", " << objectCenterY << std::endl;

      
    ROS_INFO("segment using grabcut6");	
      //cvShowImage ("erodedMask", erodedMask);
      //cvWaitKey (10);    
      //cvShowImage ("dilatedMask", dilatedMask);
      //cvWaitKey (10);    
      //cvShowImage ("maskedImage8UG", maskedImage8UG);
      //cvWaitKey (10);    
      //cvShowImage ("refinedMask", refinedMask);
      //cvWaitKey (10);    
      
      cvReleaseImage (&maskedImage8U);
      cvReleaseImage (&maskedImage8UG);
      cvReleaseImage (&dilatedMask);
      cvReleaseImage (&erodedMask);
      cvReleaseImage (&refinedMask);
    }*/
    {
      ImageConstPtr maskedImageConstPtr = boost::make_shared<Image> (perceptionData.image_proc);
      IplImage * maskedImage = bridge_.imgMsgToCv(maskedImageConstPtr);
      IplImage * maskedImage8U = cvCreateImage (cvGetSize (maskedImage), IPL_DEPTH_8U, 3);
      IplImage * maskedImage8UG = cvCreateImage (cvGetSize (maskedImage), IPL_DEPTH_8U, 1);
      cvConvert (maskedImage, maskedImage8U);
      cvCvtColor (maskedImage8U, maskedImage8UG, CV_BGR2GRAY);
      
      ROS_INFO("segment using grabcut0");	

      IplImage * resizedMaskedImage8UG = cvCreateImage (cvSize (maskedImage->width/2, maskedImage->height/2), IPL_DEPTH_8U, 1);
      IplImage * resizedColorImageClone = cvCreateImage (cvSize (maskedImage->width/2, maskedImage->height/2), IPL_DEPTH_32F, 3);
      cvResize (maskedImage8UG, resizedMaskedImage8UG);
      cvResize (colorImageClone, resizedColorImageClone);
      
      ROS_INFO("segment using grabcut1");	
      IplImage * dilatedMask = cvCreateImage (cvGetSize (resizedMaskedImage8UG), IPL_DEPTH_8U, 1);
      IplImage * erodedMask = cvCreateImage (cvGetSize (resizedMaskedImage8UG), IPL_DEPTH_8U, 1);
    
      ROS_INFO("segment using grabcut2");	
      cvDilate (resizedMaskedImage8UG, dilatedMask, NULL, 5);
      cvDilate (resizedMaskedImage8UG, erodedMask, NULL, 2);
      cvErode (erodedMask, erodedMask, NULL, 7);
      
      ROS_INFO("segment using grabcut3");	
      CvRect objectROI = cvRect (
        (perceptionData.roi[0].x_offset-50)/2,
        (perceptionData.roi[0].y_offset-50)/2,
        (perceptionData.roi[0].width+100)/2,
        (perceptionData.roi[0].height+100)/2 );
        
      if (objectROI.x < 0) objectROI.x = 0;
      if (objectROI.y < 0) objectROI.y = 0;
      if (objectROI.x >= resizedMaskedImage8UG->width) objectROI.x = resizedMaskedImage8UG->width-1;
      if (objectROI.y >= resizedMaskedImage8UG->height) objectROI.y = resizedMaskedImage8UG->height-1;
      
      ROS_INFO("segment using grabcut4");	
      IplImage * refinedMask = segmentObject (resizedColorImageClone, dilatedMask, erodedMask, objectROI);
      
      IplImage * resizedRefinedMask = cvCreateImage (cvGetSize (maskedImage), IPL_DEPTH_8U, 1);
      cvResize (refinedMask, resizedRefinedMask);
      
      
      CvScalar objectPixelCount = cvSum (resizedRefinedMask);
      
      if (objectPixelCount.val[0] > 0)
      {
        cvDilate (resizedRefinedMask, objectMask, NULL, 5);
      }
      else
      {
        cvDilate (maskedImage8UG, objectMask, NULL, 5);
      }
      
      
      ROS_INFO("segment using grabcut5");	
      // compute center of mass of refined mask
      int objectCenterCounter = 0;
      
      for (int rowIndex = 0; rowIndex < resizedRefinedMask->height; ++rowIndex)
      {
        for (int colIndex = 0; colIndex < resizedRefinedMask->width; ++colIndex)
        {
          if (CV_IMAGE_ELEM (objectMask, unsigned char, rowIndex, colIndex) != 0)
          {
            objectCenterX += colIndex;
            objectCenterY += rowIndex;
            ++objectCenterCounter;
          }
        }
      }
      if (objectCenterCounter > 0)
      {
        objectCenterX /= objectCenterCounter;
        objectCenterY /= objectCenterCounter;
      }
      
      std::cerr << objectCenterX << ", " << objectCenterY << std::endl;
      
      const int templateWidth = templateDetector1_->getTemplateWidth ();
      const int templateHeight = templateDetector1_->getTemplateHeight ();
      
      //if (objectCenterX-templateWidth/2 < 0) objectCenterX = templateWidth/2+1;
      //if (objectCenterY-templateHeight/2 < 0) objectCenterY = templateHeight/2+1;
      //if (objectCenterX+templateWidth/2 >= smoothedImage->width) objectCenterX = smoothedImage->width - (templateWidth/2+1);
      //if (objectCenterY+templateHeight/2 >= smoothedImage->height) objectCenterY = smoothedImage->height - (templateHeight/2+1);

      if (objectCenterX-templateWidth/2 < 0 
        || objectCenterY-templateHeight/2 < 0 
        || objectCenterX+templateWidth/2 >= smoothedImage->width 
        || objectCenterY+templateHeight/2 >= smoothedImage->height)
      {
        skipImage = true;
      }

      std::cerr << objectCenterX << ", " << objectCenterY << std::endl;

      
      ROS_INFO("segment using grabcut6");	
      cvShowImage ("erodedMask", erodedMask);
      cvWaitKey (10);    
      cvShowImage ("dilatedMask", dilatedMask);
      cvWaitKey (10);    
      cvShowImage ("maskedImage8UG", maskedImage8UG);
      cvWaitKey (10);    
      cvShowImage ("refinedMask", refinedMask);
      cvWaitKey (10);    
      
      cvReleaseImage (&maskedImage8U);
      cvReleaseImage (&maskedImage8UG);
      cvReleaseImage (&dilatedMask);
      cvReleaseImage (&erodedMask);
      cvReleaseImage (&refinedMask);
      
      cvReleaseImage (&resizedRefinedMask);
      cvReleaseImage (&resizedColorImageClone);
      cvReleaseImage (&resizedMaskedImage8UG);
    }
    cvShowImage ("objectMask", objectMask);
    cvWaitKey (10);    
    
    
    if (!skipImage)
    {
      // discretize gradient data
      ROS_INFO("discretize data");	
      const int regionWidth_ = 7;
      const int regionHeight_ = 7;
      
      const int horizontalSamples = grayImage->width/regionWidth_;
      const int verticalSamples = grayImage->height/regionHeight_;
      
      unsigned char * discretizedData = new unsigned char[horizontalSamples*verticalSamples];
      
      std::vector<unsigned char> discretizedDataVec (horizontalSamples*verticalSamples);
    
#ifdef USE_GRADIENTS      
#ifdef USE_RESTRICTED_NUM_OF_GRADIENTS
      GradientDiscretizer discretizer;
      discretizer.discretize(
        smoothedImage,
        objectMask,
        regionWidth_,
        regionHeight_,
        discretizedDataVec,
        1 );
#else
      GradientDiscretizer discretizer;
      discretizer.discretize(
        smoothedImage,
        objectMask,
        regionWidth_,
        regionHeight_,
        discretizedDataVec,
        8 );
#endif
#endif
#ifdef USE_SURFACE_NORMALS
      SurfaceNormalDiscretizer discretizer;
      discretizer.discretizeNonDense(
        cloudMsg,
        objectMask,
        regionWidth_,
        regionHeight_,
        smoothedImage->width,
        smoothedImage->height,
        camInfoMsg,
        discretizedDataVec );
#endif
        
      int elementIndex = 0;
      for (std::vector<unsigned char>::const_iterator iter = discretizedDataVec.begin (); 
      iter != discretizedDataVec.end (); 
      ++iter)
      {
        discretizedData[elementIndex] = *iter;
        
        ++elementIndex;
      }          
      
      
      // detect candidates
      ROS_INFO("detect candidates");	
      std::list< ::ftd::Candidate* > * candidateList = templateDetector1_->process(discretizedData, detectionThreshold_, horizontalSamples, verticalSamples);
      
      
      // process candidates
      bool objectDetected = false;
      if (candidateList != NULL)
      {
	      for (int classIndex = 0; classIndex < templateDetector1_->getNumOfClasses(); ++classIndex)
	      {			
	        // draw detected candidates
	        ROS_INFO("draw candidates");	
		      for (std::list< ::ftd::Candidate* >::iterator candidateIter = candidateList[classIndex].begin(); candidateIter != candidateList[classIndex].end(); ++candidateIter)
		      {				  
#ifndef SAVE_IMAGES  
				    drawTemplateBorder (
				      colorImage,
				      CV_RGB (0, 0, 255),
				      (*candidateIter)->getCol (),
				      (*candidateIter)->getRow (),
				      templateDetector1_->getTemplateWidth(),
				      templateDetector1_->getTemplateHeight() );
#endif // SAVE_IMAGES				      
		      }
		      
		      
		      // find best candidate
		      ROS_INFO("find best candidate");	
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
		      
		      
		      // validate candidate
		      ROS_INFO("validate candidate");	
		      bool candidateValidated = false;
          if (bestCandidateIter != candidateList[classIndex].end())
          {
            ROS_INFO ("bestCandidateResponse: %d", bestCandidateResponse);
            
            const int templateHorizontalSamples = templateDetector1_->getTemplateWidth () / templateDetector1_->getSamplingSize ();
            const int templateVerticalSamples = templateDetector1_->getTemplateHeight () / templateDetector1_->getSamplingSize ();
            
            
            ::ros::Time t_disp1 = ::ros::Time::now ();
            const int templateWidth = templateDetector1_->getTemplateWidth ();
            const int templateHeight = templateDetector1_->getTemplateHeight ();
            
            const int templateStartX = (*bestCandidateIter)->getCol ();
            const int templateStartY = (*bestCandidateIter)->getRow ();
            
            const int templateCenterX = templateStartX + templateWidth/2;
            const int templateCenterY = templateStartY + templateHeight/2;
            
            /*int validationResponse = validate (
              (*bestCandidateIter)->getIndex ()-1,
              smoothedImage,
              disparityImage,
              templateCenterX,
              templateCenterY );*/
              
            //int maskedDisparityResponse = evaluateMaskedDisparity (
            //  (*bestCandidateIter)->getIndex ()-1,
            //  (*bestCandidateIter)->getCol (),
            //  (*bestCandidateIter)->getRow (),
            //  templateHorizontalSamples,
            //  templateVerticalSamples,
            //  templateDetector1_->getSamplingSize (),
            //  templateDetector1_->getSamplingSize (),
            //  horizontalSamples,
            //  verticalSamples,
            //  colorImageClone,
            //  disparityMsg );
            ::ros::Time t_disp2 = ::ros::Time::now ();
            
            ROS_INFO ("validation time: %f", (t_disp2-t_disp1).toSec ());

            //std::cerr << "masked disparity response: " << maskedDisparityResponse << std::endl;
      
            candidateValidated = true;
            objectDetected = true;
            
#ifndef SAVE_IMAGES  
				    drawTemplateBorder (
				      colorImage,
				      CV_RGB (0, 255, 0),
				      (*bestCandidateIter)->getCol (),
				      (*bestCandidateIter)->getRow (),
				      templateDetector1_->getTemplateWidth(),
				      templateDetector1_->getTemplateHeight() );     
				              
			      templateDetector1_->drawContour ((*bestCandidateIter)->getIndex ()-1, colorImage, templateStartX, templateStartY);
#endif // SAVE_IMAGES
          }
		      
		      
		      // if best response is below learning threshold then learn a new template at this position
          if ( bestCandidateIter != candidateList[classIndex].end()
            && bestCandidateResponse < learningThreshold_ )
          {
            if (addTemplate (objectCenterX, objectCenterY, smoothedImage, objectMask, discretizedData, horizontalSamples, verticalSamples))
            {
              addValidationData (smoothedImage, disparityImage, objectMask, objectCenterX, objectCenterY, perceptionData.cloud);
              
              //saveTemplates (templateFileName1_, templateFileName2_);
              //ROS_INFO("Templates saved");
            }
          }
		    }

	      for (int classIndex = 0; classIndex < templateDetector1_->getNumOfClasses(); ++classIndex)
	      {
		      ::ftd::emptyPointerList(candidateList[classIndex]);
	      }
	    }
	    
	    
	    if (!objectDetected)
      {
        ROS_INFO("object not detected, learn new template");	
        
        if (addTemplate (objectCenterX, objectCenterY, smoothedImage, objectMask, discretizedData, horizontalSamples, verticalSamples))
        {
          addValidationData (smoothedImage, disparityImage, objectMask, objectCenterX, objectCenterY, perceptionData.cloud);
          
          //saveTemplates (templateFileName1_, templateFileName2_);
          //ROS_INFO("Templates saved");          
        }
      }    
      
      delete[] discretizedData;
    }

    cvReleaseImage (&disparityImage);
    cvReleaseImage (&objectMask);
  }
  
  ROS_INFO ("num of templates: %d, %d, %d", templateDetector1_->getNumOfTemplates (), disparityTemplates_.size (), templateDetector1_->contours_.size ());


  cvScale (colorImage, colorImage, 1.0f/255.0f);
  cvShowImage ("colorImage", colorImage);
  int pressedKey = cvWaitKey(10);

  if (pressedKey == 27)
  {
    stop_ = true;
  }
  

#ifdef SAVE_IMAGES  
  {
    cvScale (colorImage, colorImage, 255.0);

    std::stringstream ss;
    ss << "bimodal_adaptive_template_learner_" << imageCounter_ << ".png";
    
    cvSaveImage (ss.str ().c_str (), colorImage);
    ++imageCounter_;
  }  
#endif // SAVE_IMAGES
 

  cvReleaseImage (&colorImage);
  cvReleaseImage (&colorImageClone);
  cvReleaseImage (&grayImage);
  cvReleaseImage (&smoothedImage);
  
  
  processed_ = true;    
  
  ROS_INFO ("processing finished");
}






