/* 
 * File:   Coordinate3DProviderDenseStereo.h
 * Author: gedikli
 *
 * Created on 29. April 2010, 19:45
 */

#ifndef STEREO_OBJECT_RECOGNITION_COORDINATE3D_PROVIDER_DENSE_STEREO_H
#define	STEREO_OBJECT_RECOGNITION_COORDINATE3D_PROVIDER_DENSE_STEREO_H
#include <stereo_object_recognition/Coordinate3DProvider.h>
#include <stereo_msgs/DisparityImage.h>
#include <sensor_msgs/PointCloud2.h>
#include <image_geometry/stereo_camera_model.h>

namespace stereo_object_recognition
{
  class Coordinate3DProviderDenseStereo : public Coordinate3DProvider
  {
    public:
      Coordinate3DProviderDenseStereo( const stereo_msgs::DisparityImage& disparity_image,
                                       const image_geometry::StereoCameraModel& stereo_camera_model );
      Coordinate3DProviderDenseStereo( const Coordinate3DProviderDenseStereo& orig );
      virtual ~Coordinate3DProviderDenseStereo();
      virtual Coordinate3DProvider* clone() const;
      virtual unsigned getWidth() const;
      virtual unsigned getHeight() const;
      virtual bool getCoordinate( const cv::Point2d& image_point, cv::Point3d& coordinate ) const;
      virtual void getPointCloud( std::vector<cv::Point3d>& cloud, const cv::Mat& mask = cv::Mat() ) const;
    protected:
      //image_geometry::StereoCameraModel _stereoCameraModel;
      cv::Mat_<cv::Vec3f> dense_points_;
      cv::Size image_size_;
  };
} //namespace stereo_object_recognition
#endif	/* STEREO_OBJECT_RECOGNITION_COORDINATE3D_PROVIDER_DENSE_STEREO_H */

