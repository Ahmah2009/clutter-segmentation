/* 
 * File:   Coordinate3DProvider.h
 * \author Suat Gedikli
 * \date 28. April 2010
 */

#ifndef STEREO_OBJECT_RECOGNITION_COORDINATE3D_PROVIDER_H
#define	STEREO_OBJECT_RECOGNITION_COORDINATE3D_PROVIDER_H

#include <opencv/cv.h>
#include <pcl/point_cloud.h>
#include <geometry_msgs/Point32.h>

namespace stereo_object_recognition
{
  class Coordinate3DProvider
  {
    public:
    /**
     * \author Suat Gedikli
     * \date 28. April 2010
     * \brief virtual desctructor
     */
    virtual ~Coordinate3DProvider()
    {
    }

    /**
     * \author Suat Gedikli
     * \date 28. April 2010
     * \brief polymorphic clone method
     * \return the cloned object
     */
    virtual Coordinate3DProvider* clone() const = 0;

    /**
     * \author Suat Gedikli
     * \date 5. Aug 2010
     * \brief returns the width of the left image of the input stereo frame. If not from stereo => 0
     * \return the width of the left image or 0 if coordinates are not from stereo.
     */
    virtual unsigned getWidth() const
    {
      return 0;
    }

    /**
     * \author Suat Gedikli
     * \date 5. Aug 2010
     * \brief returns the height of the left image of the input stereo frame. If not from stereo => 0
     * \return the height of the left image or 0 if coordinates are not from stereo.
     */
    virtual unsigned getHeight() const
    {
      return 0;
    }

    /**
     * \author Suat Gedikli
     * \date 5. Aug 2010
     * \brief returns wheter the coordinates are dense, thus the pointcloud is complete, false otherwise
     * \return true if coordinates are dense, thus the pointcloud is complete, false otherwise
     */
    virtual bool isDense() const
    {
      return false;
    }

    /**
     * \author Suat Gedikli
     * \date 28. April 2010
     * \brief Provides the 3D coordinate in camera coordinate system of a image point
     * \param[in] imagePoint image point
     * \param[out] coordinate the according 3D coorinate of the image point in camera coordiante frame
     * \return false if no coordinate for the input pixel could be determined, true otherwise
     */
    virtual bool getCoordinate( const cv::Point2d& image_point, cv::Point3d& coordinate ) const = 0;

    /**
     * \author Suat Gedikli
     * \date 28. April 2010
     * \brief Provides the 3D point cloud for all pixels with non-zero elements in mask, or full point cloud if mask is empty
     * \param[in] mask
     * \return point cloud
     */
    virtual void getPointCloud( std::vector<cv::Point3d>& cloud, const cv::Mat& mask = cv::Mat() ) const = 0;

    /**
     * \author Suat Gedikli
     * \date 28. April 2010
     * \brief Provides the 3D point cloud for all pixels with non-zero elements in mask, or full point cloud if mask is empty
     * \param[in] mask
     * \return point cloud
     */
    template<class PointT>
    void getPointCloud( pcl::PointCloud<PointT>& cloud, const cv::Mat& mask = cv::Mat() ) const;
  };

  template<class PointT>
  void Coordinate3DProvider::getPointCloud( pcl::PointCloud<PointT>& point_cloud, const cv::Mat& mask ) const
  {
    std::vector<cv::Point3d> cloud;
    getPointCloud( cloud, mask );
    point_cloud.width = getWidth();
    point_cloud.height = getHeight();
    point_cloud.is_dense = isDense();

    point_cloud.points.resize( cloud.size() );
    for( unsigned idx = 0; idx < cloud.size(); ++idx )
    {
      point_cloud.points[idx].x = cloud[idx].x;
      point_cloud.points[idx].y = cloud[idx].y;
      point_cloud.points[idx].z = cloud[idx].z;
    }
  }
} //namespace stereo_object_recognition
#endif	/* STEREO_OBJECT_RECOGNITION_COORDINATE3D_PROVIDER_H */

