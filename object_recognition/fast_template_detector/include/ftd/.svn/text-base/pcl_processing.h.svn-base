/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id: pcl_processing.h 29038 2010-04-23 16:44:44Z rusu $
 *
 */

#ifndef _PCL_PROCESSING_
#define _PCL_PROCESSING_

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/pcl_base.h>

#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_ann.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/segmentation/extract_clusters.h>

#include "fast_template_detector/PerceptionData.h"

#include <iostream>
#include <fstream>

namespace ftd
{
  
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Write point cloud data to a stream
    * \param fileStream the stream
    * \param cloud the point cloud data message
    */
  void
    writePointCloud (std::ofstream & fileStream, sensor_msgs::PointCloud2 &cloud);
    
    
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Read point cloud data from a stream
    * \param fileStream the stream
    * \param cloud the point cloud data message
    */
  sensor_msgs::PointCloud2
    readPointCloud (std::ifstream & fileStream);
    

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief ExtractRegions extracts a set of regions of interest in a PCD/Image. The assumption here is that we are
    * looking for objects supported by large planar structures.
    */
  class ExtractObjectRegions : public pcl::PCLBase<pcl::PointXYZ>
  {
    typedef sensor_msgs::CameraInfo CameraInfo;
    typedef CameraInfo::Ptr CameraInfoPtr;
    typedef CameraInfo::ConstPtr CameraInfoConstPtr;

    typedef sensor_msgs::Image Image;
    typedef Image::Ptr ImagePtr;
    typedef Image::ConstPtr ImageConstPtr;

    typedef pcl::PointXYZ Point;
    typedef pcl::PointCloud<Point> PointCloud;
    typedef PointCloud::Ptr PointCloudPtr;
    typedef PointCloud::ConstPtr PointCloudConstPtr;
    typedef pcl::KdTreeANN<Point>::Ptr KdTreePtr;

    public:
      ExtractObjectRegions ()
      {
        // Default parameters for PCL objects
        grid_.setLeafSize (0.01, 0.01, 0.01);             // 1cm downsampling size
        grid_.setFilterFieldName ("z");
        grid_.setFilterLimits (0.4, 1.5);                 // only points between 0.4 and 1.6m from the camera
        grid_.setDownsampleAllData (false);

        normals_tree_ = boost::make_shared<pcl::KdTreeANN<Point> > ();
        n3d_.setRadiusSearch (0.02);                      // normals are estimated using a sphere with a radius of 2cm
        n3d_.setSearchMethod (normals_tree_);

        seg_.setDistanceThreshold (0.1);                  // inliers must have a distance to the plane < 0.1m
        seg_.setMaxIterations (10000);
        seg_.setNormalDistanceWeight (0.1);
        seg_.setOptimizeCoefficients (true);
        seg_.setModelType (pcl::SACMODEL_NORMAL_PLANE);
        seg_.setMethodType (pcl::SAC_RANSAC);

        proj_.setModelType (pcl::SACMODEL_NORMAL_PLANE);

        prism_.setHeightLimits (0.01, 0.5);               // objects must be between 1mm and 0.3m from the table plane

        clusters_tree_ = boost::make_shared<pcl::KdTreeANN<Point> > ();
        cluster_.setClusterTolerance (0.04);              // 4cm between two objects
        cluster_.setMinClusterSize (100);                 // 100 points minimum per object
        cluster_.setSearchMethod (clusters_tree_);
      }

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Provide a pointer to the input camera info
        * \param cam_info the CameraInfo message
        */
      inline void
        setInputCameraInfo (const CameraInfoConstPtr &cam_info)
      {
        cam_info_ = cam_info;
      }

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Provide a pointer to the input image
        * \param image the input rectified image
        */
      inline void
        setInputImage (const ImageConstPtr &img)
      {
        image_ = img;
      }
      
			/** \brief Provide a pointer to the input camera info
        * \param image the input camera info
        */
      inline void
        setInputCamInfo (const CameraInfoConstPtr &cam_info)
      {
        cam_info_ = cam_info;
			}

      bool extract (fast_template_detector::PerceptionData &data, bool usePanTiltInformation = false);
      bool extractTableMask (fast_template_detector::PerceptionData &data);

    private:
      // --[ PCL Objects
      KdTreePtr normals_tree_, clusters_tree_;
      pcl::VoxelGrid<Point> grid_;
      pcl::NormalEstimation<Point, pcl::Normal> n3d_;
      pcl::SACSegmentationFromNormals<Point, pcl::Normal> seg_;
      pcl::ProjectInliers<Point> proj_;
      pcl::ConvexHull2D<Point, Point> hull_;
      pcl::ExtractIndices<Point> ex_;
      pcl::ExtractPolygonalPrismData<Point> prism_;
      pcl::EuclideanClusterExtraction<Point> cluster_;

      /** \brief The input rectified image. */
      ImageConstPtr image_;
      
      /** \brief The input camera info. */
      CameraInfoConstPtr cam_info_;

      // --[ PCL intermediate results
      // The downsampled point cloud data
      PointCloudConstPtr cloud_downsampled_;
      // The resultant estimated point cloud normals for \a cloud_downsampled_
      pcl::PointCloud<pcl::Normal>::ConstPtr cloud_normals_;
      // The vector of indices from cloud_downsampled_ that represents the planar table component
      pcl::PointIndices::ConstPtr table_inliers_;
      // The model coefficients of the planar table component
      pcl::ModelCoefficients::ConstPtr table_coefficients_;
      // The set of point inliers projected on the planar table component from \a cloud_downsampled_
      PointCloudConstPtr table_projected_;
      // The convex hull of \a table_projected_
      PointCloudConstPtr table_hull_;
      // All the points in \a cloud_downsampled_ that do not belong to the table plane
      PointCloudConstPtr all_minus_table_;
      // The vector of indices from cloud_downsampled_ that represents potential object indices
      pcl::PointIndices::ConstPtr cloud_objects_indices_;
      // The remaining of the \a cloud_downsampled_ which lies inside the \a table_hull_ polygon
      PointCloudConstPtr cloud_objects_;

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Get a string representation of the name of this class. */
      std::string getName () const { return ("ExtractObjectRegions"); }
      
      void get2DProjectionFrom3D (const Eigen3::Vector4f &min_pt, const Eigen3::Vector4f &max_pt, const sensor_msgs::CameraInfoConstPtr &cam_info, sensor_msgs::RegionOfInterest &roi);
      void get2DProjectionFrom3D (const Eigen3::Vector4f &min_pt, const Eigen3::Vector4f &max_pt, const sensor_msgs::CameraInfoConstPtr &cam_info, sensor_msgs::RegionOfInterest &roi, IplImage * mask);
  };

      
      
}

#endif
