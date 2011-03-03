//////////////////////////////////////////////////////////////////////////////
// 
// Authors: Stefan Holzer 2010 
// Version: 1.0 
//  
//////////////////////////////////////////////////////////////////////////////

#ifndef FTD_LEARNERS_BIMODAL_ADAPTIVE_TEMPLATE_LEARNER_H_
#define FTD_LEARNERS_BIMODAL_ADAPTIVE_TEMPLATE_LEARNER_H_ FTD_LEARNERS_BIMODAL_ADAPTIVE_TEMPLATE_LEARNER_H_

#include <ftd/util.h>
#include <ftd/fast_template_detector_vs.h>
#include <ftd/fast_template_detector_vs.h>
#include <dot/DiscretizedData.h>
#include <dot/DetectionCandidates.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <io/mouse.h>
#include <util/timer.h>
#include <stereo_msgs/DisparityImage.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <emmintrin.h>

#include <fstream>
#include <iostream>
#include <vector>
#include <list>

#include "pcl/compute_normals.h"
#include <pcl/io/pcd_io.h>

#include <cv_bridge/CvBridge.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "ftd/pcl_processing.h"

// BAG
#include <rosrecord/Player.h>

//#include <rosbag/bag.h>
//#include <rosbag/view.h>
#include <topic_tools/shape_shifter.h>


#define SAVE_IMAGES


typedef ::message_filters::sync_policies::ApproximateTime< ::sensor_msgs::Image, ::sensor_msgs::CameraInfo, ::sensor_msgs::PointCloud2, stereo_msgs::DisparityImage > AdaptiveTemplateLearnerBAGSyncPolicy;


/**
 * \namespace Namespace ftd for fast template detection.
 * \brief Namespace for fast template detection.
 */
namespace ftd
{

  /**
   * \namespace Namespace for learners.
   * \brief Namespace for learners.
   */
  namespace learners
  {
 
    class BimodalAdaptiveTemplateLearner
    {
        
    public: // functions
      
      BimodalAdaptiveTemplateLearner (
        const int templateWidth,
        const int templateHeight,
        const int regionSize,
        const int numOfCharsPerElement );
      virtual ~BimodalAdaptiveTemplateLearner ();  
      
      /**
       * Processes the next frame.
       */
      void
        processNextFrame (
          std::vector<unsigned char> & discretizedData,
          const int regionWidth,
          const int regionHeight,
          const int horizontalSamples,
          const int verticalSamples,
          const int threshold_,
          const int learnThreshold_ );
                  
      void
        addTemplate (
          std::vector<unsigned char> & discretizedData,
          const int templateCenterX,
          const int templateCenterY,
          const int regionWidth,
          const int regionHeight,
          const int horizontalSamples,
          const int verticalSamples,
          const int classIndex );
          
      void
        loadTemplates (
          std::string & fileName1,
          std::string & fileName2 );
          
      void
        saveTemplates (
          std::string & fileName1,
          std::string & fileName2 );
         
    protected:
      
      ::ftd::FastTemplateDetectorVS * templateDetector1_;
      ::ftd::FastTemplateDetectorVS * templateDetector2_;
 
#ifdef SAVE_IMAGES
      int imageCounter_;
#endif SAVE_IMAGES      

    };
  
  
  
    class BimodalAdaptiveTemplateLearnerBAG
      : public BimodalAdaptiveTemplateLearner
    {
    
    public: // datatyps
    
      enum DiscretizationType
      {
        GRADIENT,
        SURFACE_NORMAL,
        MASKED_DISPARITY
      };
        
    public: // functions
      
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
        const int learningThreshold );
      virtual ~BimodalAdaptiveTemplateLearnerBAG ();  
      
      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Spin and get data from the BAG file. */
      void
        spin ()
      {
        if (!init_)
          return;

        // Spin
        while (nextMsg ())
        {
          if (!processed_)
            continue;

          processed_ = false;
        }
      }      
      
      /*void
        handlerImage (
          const ::sensor_msgs::ImageConstPtr & msg );
      
      void
        handlerDiscretizedData (
          const ::dot::DiscretizedDataConstPtr & msg );*/
          
      void
        callback (
          const ::sensor_msgs::ImageConstPtr & imageMsg,
          const ::sensor_msgs::CameraInfoConstPtr & camInfoMsg,
          const ::sensor_msgs::PointCloud2ConstPtr & cloudMsg,
          const ::stereo_msgs::DisparityImageConstPtr & disparityMsg );          
      void
        callback2 (
          const ::sensor_msgs::ImageConstPtr & imageMsg,
          const ::sensor_msgs::CameraInfoConstPtr & camInfoMsg,
          const ::sensor_msgs::PointCloud2ConstPtr & cloudMsg,
          const ::stereo_msgs::DisparityImageConstPtr & disparityMsg );
          
      void
        loadTemplates (
          std::string & fileName1,
          std::string & fileName2 );
          
      void
        saveTemplates (
          std::string & fileName1,
          std::string & fileName2 );          
       
    private: // functions
    
      IplImage *
        getColorImage32F ( 
          IplImage * image );
    
      IplImage *
        getGrayImage32F ( 
          IplImage * image );
          
      IplImage *
        getDisparityImage ( 
          const ::stereo_msgs::DisparityImageConstPtr & disparityMsg );
          
      void
        drawTemplateBorder ( 
          IplImage * image,
          CvScalar color,
          const int templateStartX,
          const int templateStartY,
          const int templateWidth,
          const int templateHeight );
    
      void
        computeROIsFromPointCloud (
          const ::sensor_msgs::ImageConstPtr & imageMsg,
          const ::sensor_msgs::CameraInfoConstPtr & camInfoMsg,
          const ::sensor_msgs::PointCloud2ConstPtr & cloudMsg,
          dot::PerceptionData & perceptionData );
          
      bool    
        addTemplate (
          const int templateCenterX,
          const int templateCenterY,
          IplImage * smoothedImage,
          IplImage * objectMask,
          unsigned char * discretizedData,
          const int horizontalSamples,
          const int verticalSamples );
          
      void
        addValidationData (
          IplImage * smoothedImage,
          IplImage * disparityImage,
          IplImage * mask,
          const int objectCenterX,
          const int objectCenterY,
          sensor_msgs::PointCloud2 & cloud );
          
      int
        validate (
          const int templateId,
          IplImage * smoothedImage,
          IplImage * disparityImage,
          const int objectCenterX,
          const int objectCenterY );
    
      int 
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
          IplImage * colorImage,
          const ::stereo_msgs::DisparityImageConstPtr & disparityMsg );
    
      static IplImage *
        segmentObject (
          IplImage * colorImage,
          IplImage * dilatedMask,
          IplImage * erodedMask,
          CvRect & roi );
          
              
      static CvRect
        estimateROI (
          IplImage * smoothedImage,
          CvRect & initialRegion );
    
      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Get the next PointCloud2 and Image messages from the BAG file. */
      inline bool
        nextMsg ()
      {
        if (!init_)
          return (false);
          
        if (stop_)
          return (false);

        bool res;
        while ((res = player_.nextMsg ()) && !(imageReceived_ && camInfoReceived_ && cloudReceived_ && disparityReceived_));
        imageReceived_ = false;
        camInfoReceived_ = false;
        cloudReceived_ = false;
        disparityReceived_ = false;

        return (res);
      }
          
			//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Get the next CameraInfo messages from the BAG file. */
//      inline bool
//        nextCameraInfoMsg ()
//      {
//        if (!init_)
//          return (false);
//
//        bool res;
//        while ((res = player_.nextMsg ()) && !(camera_info_received_));
//
//        return (res);
//      }

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief The Image BAG message handler.
        * \param topic_name the name of the topic
        * \param image the resultant output message
        * \param t
        * \param t_shift
        * \param n
        */
      static void
        handlerImage (std::string topic_name, ::sensor_msgs::Image * image, ros::Time t, ros::Time t_shift, void *n)
      {
        BimodalAdaptiveTemplateLearnerBAG * p = (BimodalAdaptiveTemplateLearnerBAG*) n;
        p->image_.reset (new sensor_msgs::Image (*image));
        p->sync_.add<0>(p->image_);
        p->imageReceived_ = true;
        
        ROS_INFO ("image received");
      }    
       
      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief The CamInfo BAG message handler.
        * \param topic_name the name of the topic
        * \param image the resultant output message
        * \param t
        * \param t_shift
        * \param n
        */
      static void
        handlerCamInfo (std::string topic_name, ::sensor_msgs::CameraInfo * camInfo, ros::Time t, ros::Time t_shift, void *n)
      {
        BimodalAdaptiveTemplateLearnerBAG * p = (BimodalAdaptiveTemplateLearnerBAG*) n;
        p->camInfo_.reset (new sensor_msgs::CameraInfo (*camInfo));
        p->sync_.add<1>(p->camInfo_);
        p->camInfoReceived_ = true;
        
        ROS_INFO ("camInfo received");
      }    
       
      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief The DiscretizedData BAG message handler.
        * \param topic_name the name of the topic
        * \param image the resultant output message
        * \param t
        * \param t_shift
        * \param n
        */
      /*static void
        handlerDiscretizedData (std::string topic_name, ::dot::DiscretizedData * discretizedData, ros::Time t, ros::Time t_shift, void *n)
      {
        BimodalAdaptiveTemplateLearnerBAG * p = (BimodalAdaptiveTemplateLearnerBAG*) n;
        p->discretizedData_.reset (new ::dot::DiscretizedData (*discretizedData));
        p->sync_.add<1>(p->discretizedData_);
        p->discretizedDataReceived_ = true;
      } */   
      
      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief The PointCloud2 BAG message handler.
        * \param topic_name the name of the topic
        * \param cloud the resultant output message
        * \param t
        * \param t_shift
        * \param n
        */
      static inline void
        handlerPointCloud2 (std::string topic_name, sensor_msgs::PointCloud2 *cloud, ros::Time t, ros::Time t_shift, void *n)
      {
        BimodalAdaptiveTemplateLearnerBAG *p = (BimodalAdaptiveTemplateLearnerBAG*)n;
        p->cloud_.reset (new sensor_msgs::PointCloud2 (*cloud));
        p->sync_.add<2>(p->cloud_);
        p->cloudReceived_ = true;

        ROS_INFO ("point cloud received");
      }      
       
      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief The PointCloud2 BAG message handler.
        * \param topic_name the name of the topic
        * \param cloud the resultant output message
        * \param t
        * \param t_shift
        * \param n
        */
      static inline void
        handlerDisparityImage (std::string topic_name, stereo_msgs::DisparityImage *disparity, ros::Time t, ros::Time t_shift, void *n)
      {
        BimodalAdaptiveTemplateLearnerBAG *p = (BimodalAdaptiveTemplateLearnerBAG*)n;
        p->disparity_.reset (new stereo_msgs::DisparityImage (*disparity));
        p->sync_.add<3>(p->disparity_);
        p->disparityReceived_ = true;

        ROS_INFO ("point cloud received");
      }      
       
    private: // data
    
      /** \brief The BAG player object. */
      ros::record::Player player_;
      
      /** Synchronizer */
      message_filters::Synchronizer<AdaptiveTemplateLearnerBAGSyncPolicy> sync_;
      
      /** \brief The output image dataset. */
      sensor_msgs::ImageConstPtr image_;

      /** \brief The output camera info dataset. */
      sensor_msgs::CameraInfoConstPtr camInfo_;

      /** \brief The output image dataset. */
      dot::DiscretizedDataConstPtr discretizedData_;
      
      /** \brief The output point cloud dataset. */
      sensor_msgs::PointCloud2ConstPtr cloud_;
      /** \brief The output point cloud dataset. */
      stereo_msgs::DisparityImageConstPtr disparity_;


      /** \brief Indicates whether the system has been initialized. */
      bool init_;
      /** \brief Indicates whether an image has been received. */
      bool imageReceived_;
      /** \brief Indicates whether discretized data has been received. */
      bool discretizedDataReceived_;
      /** \brief Indicates whether point cloud data has been received. */
      bool cloudReceived_;
      bool disparityReceived_;
      bool camInfoReceived_;
      
      bool processed_;
      
      bool stop_;
      
      
      std::string templateFileName1_;
      std::string templateFileName2_;
      
      ::sensor_msgs::CvBridge bridge_;
      
      int detectionThreshold_;
      int learningThreshold_;
      
      
      std::vector<IplImage*> sobelDxTemplates_;
      std::vector<IplImage*> sobelDyTemplates_;
      std::vector<IplImage*> sobelMagnitudeTemplates_;
      std::vector<IplImage*> disparityTemplates_;
      
      std::vector< sensor_msgs::PointCloud2 > pointCloudTemplates_;
     
      
      
      //rosbag::Bag bag_;
      //rosbag::View view_;
      
      //::message_filters::Synchronizer<AdaptiveTemplateLearnerBAGSyncPolicy> sync_;
      
    };
  
  }
}

#endif // FTD_LEARNERS_BIMODAL_ADAPTIVE_TEMPLATE_LEARNER_H_

