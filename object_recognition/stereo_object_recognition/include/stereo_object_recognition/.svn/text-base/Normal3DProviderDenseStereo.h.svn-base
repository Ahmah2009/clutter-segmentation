/* 
 * File:   Normal3DProviderDenseStereo.h
 * Author: gedikli
 *
 * Created on 29. April 2010, 19:45
 */

#ifndef STEREO_OBJECT_RECOGNITION_NORMAL3D_PROVIDER_DENSE_STEREO_H
#define	STEREO_OBJECT_RECOGNITION_NORMAL3D_PROVIDER_DENSE_STEREO_H
#include "Normal3DProvider.h"
#include <stereo_msgs/DisparityImage.h>
#include <sensor_msgs/PointCloud2.h>
#include <image_geometry/stereo_camera_model.h>

namespace stereo_object_recognition
{
  class Normal3DProviderDenseStereo : public Normal3DProvider
  {
    public:
      Normal3DProviderDenseStereo( const stereo_msgs::DisparityImage& disparity_image, const image_geometry::StereoCameraModel& stereo_camera_model, double radius = 0.01 /*1cm*/, double epsilon = 0.1 );
      Normal3DProviderDenseStereo( const Normal3DProviderDenseStereo& orig );
      virtual ~Normal3DProviderDenseStereo();
      virtual Normal3DProvider* clone() const;
      virtual bool getNormal( const cv::Point2d& image_point, cv::Vec3d& normal ) const;
     protected:
      mutable cv::Mat_<cv::Vec3f> dense_points_;
      const double radius_Sqr_;
      const double epsilon_;
  };
} //namespace stereo_object_recognition
#endif	/* STEREO_OBJECT_RECOGNITION_NORMAL3D_PROVIDER_DENSE_STEREO_H */

