/* 
 * File:   Coordinate3DProviderDenseStereo.cpp
 * Author: gedikli
 * 
 * Created on 29. April 2010, 19:45
 */

#include "stereo_object_recognition/Coordinate3DProviderDenseStereo.h"
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/image_encodings.h>
#include <iostream>
using std::vector;
using pcl::PointCloud;
using geometry_msgs::Point32;
using namespace cv;

namespace stereo_object_recognition
{
  Coordinate3DProviderDenseStereo::Coordinate3DProviderDenseStereo( const stereo_msgs::DisparityImage& disparity_image, const image_geometry::StereoCameraModel& stereo_camera_model )
    : dense_points_( )
    , image_size_( stereo_camera_model.left().width(), stereo_camera_model.left().height() )
  {
    const sensor_msgs::Image& dimage = disparity_image.image;
    const Mat_<float> dmat(dimage.height, dimage.width, (float*)&dimage.data[0], dimage.step);
    stereo_camera_model.projectDisparityImageTo3d(dmat, dense_points_, true);
  }

  Coordinate3DProviderDenseStereo::Coordinate3DProviderDenseStereo(const Coordinate3DProviderDenseStereo& orig)
    : dense_points_( orig.dense_points_ )
  {
  }

  Coordinate3DProviderDenseStereo::~Coordinate3DProviderDenseStereo()
  {
    // do nothing!
  }

  Coordinate3DProvider* Coordinate3DProviderDenseStereo::clone() const
  {
    return new Coordinate3DProviderDenseStereo( *this );
  }

  unsigned Coordinate3DProviderDenseStereo::getWidth() const
  {
    return (unsigned)image_size_.height;
  }

  unsigned Coordinate3DProviderDenseStereo::getHeight() const
  {
    return (unsigned)image_size_.width;
  }

  bool Coordinate3DProviderDenseStereo::getCoordinate( const Point2d& imagePointRect, Point3d& coordinate ) const
  {
    Vec3f coord = dense_points_( (int)(imagePointRect.y+.5), (int)(imagePointRect.x+.5) );

    if( coord[2] == image_geometry::StereoCameraModel::MISSING_Z)
      return false;

    coordinate = Point3d( coord );
    return true;
  }

  void Coordinate3DProviderDenseStereo::getPointCloud( vector<Point3d>& cloud, const Mat& mask ) const
  {
    if( !mask.empty() )
    {
      MatConstIterator_<unsigned char> it = mask.begin<unsigned char>();
      MatConstIterator_<unsigned char> itEnd = mask.end<unsigned char>();
      for( ; it != itEnd; ++it )
      {
        if( *it != 0 )
        {
          cloud.push_back( Point3d(dense_points_( it.pos() )) );
        }
      }
    }
    else
    {
      MatConstIterator_<Vec3f> it = dense_points_.begin();
      MatConstIterator_<Vec3f> itEnd = dense_points_.end();
      for( ; it != itEnd; ++it )
      {
        cloud.push_back( Point3d( *it ) );
      }
    }
  }
} // namespace stereo_object_recognition