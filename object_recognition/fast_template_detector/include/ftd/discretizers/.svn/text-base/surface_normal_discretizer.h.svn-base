//////////////////////////////////////////////////////////////////////////////
// 
// Authors: Stefan Holzer 2010 
// Version: 1.0 
//  
//////////////////////////////////////////////////////////////////////////////

#ifndef FTD_DISCRETIZERS_SURFACE_NORMAL_DISCRETIZER_H_
#define FTD_DISCRETIZERS_SURFACE_NORMAL_DISCRETIZER_H_ FTD_DISCRETIZERS_SURFACE_NORMAL_DISCRETIZER_H_

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <ftd/fast_template_detector_vs.h>
#include <ftd/fast_template_detector_vs.h>
#include <fast_template_detector/DiscretizedData.h>
#include <fast_template_detector/DetectionCandidates.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <io/mouse.h>
#include <util/timer.h>
#include <stereo_msgs/DisparityImage.h>

#include <emmintrin.h>

#include <fstream>
#include <iostream>
#include <vector>
#include <list>

#include "pcl/compute_normals.h"
#include <pcl/io/pcd_io.h>

#include <stereo_msgs/DisparityImage.h>


/**
 * \namespace Namespace ftd for fast template detection.
 * \brief Namespace for fast template detection.
 */
namespace ftd
{

  /**
   * \namespace Namespace for discretizers.
   * \brief Namespace for discretizers.
   */
  namespace discretizers
  {
 
    class SurfaceNormalDiscretizer
    {
        
    public: // functions
      
      SurfaceNormalDiscretizer ();
      virtual ~SurfaceNormalDiscretizer ();  
      
      /**
       * Discretizes the surface normals.
       */
      static void
        discretize (
          const ::sensor_msgs::PointCloud2ConstPtr & cloud2,
          const int regionWidth,
          const int regionHeight,
          unsigned char * discretizedData );
          
      /**
       * Discretizes the surface normals.
       */
      static void
        discretize (
          const ::sensor_msgs::PointCloud2ConstPtr & cloud2,
          const int regionWidth,
          const int regionHeight,
          std::vector<unsigned char> & discretizedData );

      static void
        discretize (
          const ::sensor_msgs::PointCloud2ConstPtr & cloud2,
          IplImage * mask,
          const int regionWidth,
          const int regionHeight,
          std::vector<unsigned char> & discretizedData );
          
      static void
        discretizeNonDense (
          const ::sensor_msgs::PointCloud2ConstPtr & cloud2,
          IplImage * mask,
          const int regionWidth,
          const int regionHeight,
          const int imageWidth,
          const int imageHeight,
          const ::sensor_msgs::CameraInfoConstPtr & camInfoMsg,
          std::vector<unsigned char> & discretizedData );        

      /**
       * Discretizes the surface normals.
       */
      static void
        discretize (
          const ::sensor_msgs::PointCloud2ConstPtr & cloud2,
          const ::stereo_msgs::DisparityImageConstPtr & disparity,
          const int regionWidth,
          const int regionHeight,
          std::vector<unsigned char> & discretizedData );
                    
    };
  
  }
}

#endif // FTD_DISCRETIZERS_SURFACE_NORMAL_DISCRETIZER_H_

