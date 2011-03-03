/* 
 * File:   Normal3DProviderDenseStereo.cpp
 * Author: gedikli
 * 
 * Created on 29. April 2010, 19:45
 */

#include "stereo_object_recognition/Normal3DProviderDenseStereo.h"
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/image_encodings.h>
#include <vector>

namespace stereo_object_recognition
{
  Normal3DProviderDenseStereo::Normal3DProviderDenseStereo( const stereo_msgs::DisparityImage& disparityImage, const image_geometry::StereoCameraModel& stereoCameraModel, double radius, double epsilon )
    : radius_Sqr_( radius * radius )
    , epsilon_( epsilon )
  {
    const sensor_msgs::Image& dimage = disparityImage.image;
    const cv::Mat_<float> dmat(dimage.height, dimage.width, (float*)&dimage.data[0], dimage.step);
    stereoCameraModel.projectDisparityImageTo3d(dmat, dense_points_, true);
  }

  Normal3DProviderDenseStereo::Normal3DProviderDenseStereo(const Normal3DProviderDenseStereo& orig)
    : dense_points_( orig.dense_points_ )
    , radius_Sqr_( orig.radius_Sqr_ )
    , epsilon_( orig.epsilon_ )
  {
  }

  Normal3DProviderDenseStereo::~Normal3DProviderDenseStereo()
  {
    // do nothing!
  }

  Normal3DProvider* Normal3DProviderDenseStereo::clone() const
  {
    return new Normal3DProviderDenseStereo( *this );
  }

  bool Normal3DProviderDenseStereo::getNormal( const cv::Point2d& imagePoint, cv::Vec3d& normal ) const
  {
    cv::Vec3f mean = dense_points_( (int)(imagePoint.x+.5), (int)(imagePoint.y+.5) );
    if( mean[2] == image_geometry::StereoCameraModel::MISSING_Z )
      return false;

    std::vector<cv::Vec3f> samples;
    cv::Mat_<cv::Vec3f>::iterator it = dense_points_.begin();

    for( cv::Mat_<cv::Vec3f>::iterator it = dense_points_.begin(); it != dense_points_.end(); ++it )
    {
      if( (*it)[2] != image_geometry::StereoCameraModel::MISSING_Z )
      {
        cv::Vec3f diff = ((*it) - mean );
        if( diff.dot( diff ) <= radius_Sqr_ )
          samples.push_back( diff );
      }
    }

    cv::Mat sampleMat = cv::Mat::zeros( 3, samples.size(), CV_64F );
    cv::Mat meanMat   = cv::Mat::zeros( 3, 1, CV_64F );

    meanMat.at<double>( 0, 0 ) = mean[0];
    meanMat.at<double>( 1, 0 ) = mean[1];
    meanMat.at<double>( 2, 0 ) = mean[2];

    for( unsigned smplIdx = 0; smplIdx < samples.size(); ++smplIdx )
    {
      sampleMat.at<double>( 0, smplIdx ) = samples[smplIdx][0];
      sampleMat.at<double>( 1, smplIdx ) = samples[smplIdx][1];
      sampleMat.at<double>( 2, smplIdx ) = samples[smplIdx][2];
    }

    cv::Mat covarMat = cv::Mat::zeros( 3, 3, CV_64F );
    cv::calcCovarMatrix( sampleMat, covarMat, meanMat, CV_COVAR_USE_AVG | CV_COVAR_COLS );

    cv::Mat_<double> eigenvectors( 3, 3 );
    cv::Mat_<double> eigenvalues( 3, 1 );
    if( !cv::eigen(covarMat, eigenvalues, eigenvectors) )
      return false;

    // if second smallest eigenvalue is bigger than 1./eps * smallest => no unique normal could be found!
    if( epsilon_ * eigenvalues( 1, 0 ) < eigenvalues( 2, 0 ) )
      return false;

    // return the smallest eigenvector
    normal[0] = eigenvectors( 2, 0 );
    normal[1] = eigenvectors( 2, 1 );
    normal[2] = eigenvectors( 2, 2 );

    return true;
  }
} // namespace stereo_object_recognition