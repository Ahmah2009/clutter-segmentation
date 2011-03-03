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
 * $Id: pcl_processing.cpp 29038 2010-04-23 16:44:44Z rusu $
 *
 */

#include <opencv/cv.h>
#include <pcl/io/pcd_io.h>
#include <ftd/pcl_processing.h>

#include <image_geometry/pinhole_camera_model.h>
#include <cv_bridge/CvBridge.h>

namespace ftd
{
  bool
    ExtractObjectRegions::extract (fast_template_detector::PerceptionData &data, bool usePanTiltInformation)
  {
    std::vector<pcl::PointIndices> clusters;
    pcl::PointCloud<Point> cloud_objects;
      
    ROS_INFO ("test1");
    if (!usePanTiltInformation)
    {
      // Downsample point cloud
      ROS_INFO ("Downsample point cloud");
      pcl::PointCloud<Point> cloud_downsampled;
      grid_.setInputCloud (input_);
      grid_.filter (cloud_downsampled);
      cloud_downsampled_.reset (new PointCloud (cloud_downsampled));

      // Estimate point normals (for planar segmentation)
      ROS_INFO ("Estimate point normals (for planar segmentation)");
      pcl::PointCloud<pcl::Normal> cloud_normals;
      n3d_.setInputCloud (cloud_downsampled_);
      n3d_.compute (cloud_normals);
      cloud_normals_.reset (new pcl::PointCloud<pcl::Normal> (cloud_normals));

      // Segment the plane
      ROS_INFO ("Segment the plane");
      pcl::PointIndices table_inliers; pcl::ModelCoefficients table_coefficients;
      seg_.setInputCloud (cloud_downsampled_);
      seg_.setInputNormals (cloud_normals_);
      seg_.segment (table_inliers, table_coefficients);
      table_inliers_.reset (new pcl::PointIndices (table_inliers));
      table_coefficients_.reset (new pcl::ModelCoefficients (table_coefficients));
      if (table_coefficients.values.size () > 3 && table_inliers.indices.size () > 4)
        ROS_INFO ("[pcl::%s::extract] Planar model found with %d inliers: [%f %f %f %f].", getName ().c_str (), (int)table_inliers.indices.size (), table_coefficients.values[0], table_coefficients.values[1], table_coefficients.values[2], table_coefficients.values[3]);
      else
        return false;

      // Extract the table
      ROS_INFO ("Extract the table");
      PointCloud table_projected;
      proj_.setInputCloud (cloud_downsampled_);
      proj_.setIndices (table_inliers_);
      proj_.setModelCoefficients (table_coefficients_);
      proj_.filter (table_projected);
      table_projected_.reset (new PointCloud (table_projected));

      // Estimate the convex hull
      ROS_INFO ("Estimate the convex hull");
      PointCloud table_hull;
      hull_.setInputCloud (table_projected_);
      hull_.reconstruct (table_hull);
      
//      point_cloud::toMsg (table_hull, data.table_hull);
      pcl::toROSMsg (table_hull, data.table_hull);
      table_hull_.reset (new pcl::PointCloud<Point> (table_hull));

      // Extract the remaining of the scene
      ROS_INFO ("Extract the remaining of the scene");
      PointCloud all_minus_table;
      ex_.setInputCloud (cloud_downsampled_);
      ex_.setIndices (table_inliers_);
      ex_.setNegative (true);
      ex_.filter (all_minus_table);
      all_minus_table_.reset (new PointCloud (all_minus_table));

      // Get the objects on top of the table
      ROS_INFO ("Get the objects on top of the table");
      pcl::PointIndices cloud_objects_indices;
      prism_.setInputCloud (all_minus_table_);
      prism_.setInputPlanarHull (table_hull_);
      prism_.segment (cloud_objects_indices);
      ROS_INFO ("Number of point candidates on the table: %d", (int)cloud_objects_indices.indices.size ());
      if (cloud_objects_indices.indices.size () == 0)
        return false;
      cloud_objects_indices_.reset (new pcl::PointIndices (cloud_objects_indices));

      // Break into clusters
      ROS_INFO ("Break into clusters");
      ex_.setInputCloud (all_minus_table_);
      ex_.setIndices (cloud_objects_indices_);
      ex_.setNegative (false);
      ex_.filter (cloud_objects);
      cloud_objects_.reset (new PointCloud (cloud_objects));

      cluster_.setInputCloud (cloud_objects_);
      cluster_.extract (clusters);
    }
    else
      clusters.resize (1);
      
    // Prepare the data result
    ROS_INFO ("Prepare the data result");
    //point_cloud::toMsg (*input_, data.cloud);
    pcl::toROSMsg (*input_, data.cloud);
    data.roi.resize (clusters.size ());
    data.image_orig = *image_;

    // Get the image data
    ROS_INFO ("Get the image data");
    sensor_msgs::CvBridge img_bridge;
    img_bridge.fromImage (*image_, "bgr8");
    IplImage *img_in = img_bridge.toIpl ();

    // Create the image mask
    ROS_INFO ("Create the image mask");
    //IplImage *img_mask = cvCreateImage (cvSize (input_->width, input_->height), IPL_DEPTH_8U, 1);
    IplImage *img_mask = cvCreateImage (cvGetSize (img_in), IPL_DEPTH_8U, 1);
    cvSetZero (img_mask);

    // Prepare the image output
    ROS_INFO ("Prepare the image output");
    IplImage *img_out = (IplImage*)cvClone (img_in);
    cvSetZero (img_out);
    
    ROS_INFO ("num of clusters: %i", clusters.size ());

    if (!usePanTiltInformation)
    {
      // For each cluster, find the bounding box (min/max)
      for (size_t i = 0; i < clusters.size (); ++i)
      {
        // Get the min/max in 3D
        Eigen3::Vector4f min_pt, max_pt;
        getMinMax3D (cloud_objects, clusters[i], min_pt, max_pt);

        // Project to 2D
        get2DProjectionFrom3D (min_pt, max_pt, cam_info_, data.roi[i]);

        //std::cerr << "COLUMNS x y z\nPOINTS 2\nDATA ascii" << std::endl;
        //std::cerr << min_pt[0] << " " << min_pt[1] << " " << min_pt[2] << std::endl;
        //std::cerr << max_pt[0] << " " << max_pt[1] << " " << max_pt[2] << std::endl;
        //pcl::io::savePCDFileBinary ("downsampled.pcd", all_minus_table);
        //pcl::io::savePCDFileBinary ("objects.pcd", cloud_objects);
        //pcl::io::savePCDFileBinary ("hull.pcd", table_hull);

        // Get the cluster points from the original point cloud
        std::vector<int> indices_in_box;
        pcl::getPointsInBox (*input_, min_pt, max_pt, indices_in_box);
        
        // Fill in the mask data
        for (size_t j = 0; j < indices_in_box.size (); ++j)
        {
          int ri = (int)(indices_in_box[j] / input_->width);
          int rc = (int)(indices_in_box[j] % input_->width);

          CV_IMAGE_ELEM (img_mask, uint8_t, ri, rc) = 255;
        }

        // Erode and Dilate
        //cvErode (img_mask, img_mask, NULL, 1);
        //cvDilate (img_mask, img_mask, NULL, 3);
        cvCopy (img_in, img_out, img_mask);
      }
    }
    else
    {
      // Get the min/max in 3D
      Eigen3::Vector4f min_pt, max_pt;
      getMinMax3D (*input_, min_pt, max_pt);

      // Project to 2D
      get2DProjectionFrom3D (min_pt, max_pt, cam_info_, data.roi[0]);

      // Get the cluster points from the original point cloud
      std::vector<int> indices_in_box;
      pcl::getPointsInBox (*input_, min_pt, max_pt, indices_in_box);
      
      
      std::cerr << "indices_in_box.size (): " << indices_in_box.size () << std::endl;
      std::cerr << "_input->points[0].x: " << input_->points[0].x << std::endl;

      
      image_geometry::PinholeCameraModel camera_model;
      camera_model.fromCameraInfo (cam_info_);

      Eigen3::MatrixXf points_3d = Eigen3::MatrixXf::Ones (4, indices_in_box.size ());
      for (int pointIndex = 0; pointIndex < points_3d.cols (); ++pointIndex)
      {
        points_3d (0, pointIndex) = input_->points[indices_in_box[pointIndex]].x;
        points_3d (1, pointIndex) = input_->points[indices_in_box[pointIndex]].y;
        points_3d (2, pointIndex) = input_->points[indices_in_box[pointIndex]].z;
      }

      Eigen3::MatrixXf camera_pose = Eigen3::MatrixXf::Identity (3, 4);
      Eigen3::MatrixXf intrinsic_mat = Eigen3::MatrixXf::Identity (3, 3);
      intrinsic_mat (0, 0) = camera_model.fx ();
      intrinsic_mat (0, 2) = camera_model.cx ();
      intrinsic_mat (1, 1) = camera_model.fy ();
      intrinsic_mat (1, 2) = camera_model.cy ();

      Eigen3::MatrixXf points_2d = intrinsic_mat * camera_pose * points_3d;
      for (int i = 0; i < points_2d.cols (); ++i)
        points_2d.col (i) /= points_2d (2, i);
      
      for (int i = 0; i < points_2d.cols (); ++i)
      {
        const int px = static_cast<int> (points_2d (0, i));
        const int py = static_cast<int> (points_2d (1, i));
        //if (px >= 0 && px < mask->width && py >= 0 && py < mask->height)
        {
          CV_IMAGE_ELEM (img_mask, uint8_t, py, px) = 255;
        }
      }
      
      // Fill in the mask data
      /*for (size_t i = 0; i < indices_in_box.size (); ++i)
      {
        int ri = (int)(indices_in_box[i] / input_->width);
        int rc = (int)(indices_in_box[i] % input_->width);

        CV_IMAGE_ELEM (img_mask, uint8_t, ri, rc) = 255;
      }*/
        
      cvCopy (img_in, img_out, img_mask);
    }
    
    // Convert to ROS format
    ROS_INFO ("Convert to ROS format");
    img_bridge.fromIpltoRosImage (img_out, data.image_proc);
    cvReleaseImage (&img_out);
    cvReleaseImage (&img_mask);
    
    return true;
  }

  bool
    ExtractObjectRegions::extractTableMask (fast_template_detector::PerceptionData &data)
  {
    std::vector<pcl::PointIndices> clusters;
    pcl::PointCloud<Point> cloud_objects;
      
    // Downsample point cloud
    pcl::PointCloud<Point> cloud_downsampled;
    grid_.setInputCloud (input_);
    grid_.filter (cloud_downsampled);
    cloud_downsampled_.reset (new PointCloud (cloud_downsampled));

    // Estimate point normals (for planar segmentation)
    pcl::PointCloud<pcl::Normal> cloud_normals;
    n3d_.setInputCloud (cloud_downsampled_);
    n3d_.compute (cloud_normals);
    cloud_normals_.reset (new pcl::PointCloud<pcl::Normal> (cloud_normals));

    // Segment the plane
    pcl::PointIndices table_inliers; pcl::ModelCoefficients table_coefficients;
    seg_.setInputCloud (cloud_downsampled_);
    seg_.setInputNormals (cloud_normals_);
    seg_.segment (table_inliers, table_coefficients);
    table_inliers_.reset (new pcl::PointIndices (table_inliers));
    table_coefficients_.reset (new pcl::ModelCoefficients (table_coefficients));
    if (table_coefficients.values.size () > 3 && table_inliers.indices.size () > 4)
      ROS_INFO ("[pcl::%s::extract] Planar model found with %d inliers: [%f %f %f %f].", getName ().c_str (), (int)table_inliers.indices.size (), table_coefficients.values[0], table_coefficients.values[1], table_coefficients.values[2], table_coefficients.values[3]);
    else
      return false;

    // Extract the table
    PointCloud table_projected;
    proj_.setInputCloud (cloud_downsampled_);
    proj_.setIndices (table_inliers_);
    proj_.setModelCoefficients (table_coefficients_);
    proj_.filter (table_projected);
    table_projected_.reset (new PointCloud (table_projected));

    // Estimate the convex hull
    PointCloud table_hull;
    hull_.setInputCloud (table_projected_);
    hull_.reconstruct (table_hull);
    //point_cloud::toMsg (table_hull, data.table_hull);
    pcl::toROSMsg (table_hull, data.table_hull);
    table_hull_.reset (new pcl::PointCloud<Point> (table_hull));

    // Extract the remaining of the scene
    PointCloud all_minus_table;
    ex_.setInputCloud (cloud_downsampled_);
    ex_.setIndices (table_inliers_);
    ex_.setNegative (true);
    ex_.filter (all_minus_table);
    all_minus_table_.reset (new PointCloud (all_minus_table));

    // Get the objects on top of the table
    pcl::PointIndices cloud_objects_indices;
    prism_.setInputCloud (all_minus_table_);
    prism_.setInputPlanarHull (table_hull_);
    prism_.segment (cloud_objects_indices);
    ROS_INFO ("Number of point candidates on the table: %d", (int)cloud_objects_indices.indices.size ());
    if (cloud_objects_indices.indices.size () == 0)
      return false;
    cloud_objects_indices_.reset (new pcl::PointIndices (cloud_objects_indices));

    // Break into clusters
    ex_.setInputCloud (all_minus_table_);
    ex_.setIndices (cloud_objects_indices_);
    ex_.setNegative (false);
    ex_.filter (cloud_objects);
    cloud_objects_.reset (new PointCloud (cloud_objects));

    cluster_.setInputCloud (cloud_objects_);
    cluster_.extract (clusters);
      
    // Prepare the data result
    //point_cloud::toMsg (*input_, data.cloud);
    pcl::toROSMsg (*input_, data.cloud);
    data.roi.resize (clusters.size ());
    data.image_orig = *image_;

    // Get the image data
    sensor_msgs::CvBridge img_bridge;
    img_bridge.fromImage (*image_, "bgr8");
    IplImage *img_in = img_bridge.toIpl ();

    // Create the image mask
    //IplImage *img_mask = cvCreateImage (cvSize (input_->width, input_->height), IPL_DEPTH_8U, 1);
    IplImage *img_mask = cvCreateImage (cvGetSize (img_in), IPL_DEPTH_8U, 1);
    cvSetZero (img_mask);

    // Prepare the image output
    IplImage *img_out = (IplImage*)cvClone (img_in);
    cvSetZero (img_out);

    // For each cluster, find the bounding box (min/max)
    for (size_t i = 0; i < clusters.size (); ++i)
    {
      // Get the min/max in 3D
      Eigen3::Vector4f min_pt, max_pt;
      getMinMax3D (cloud_objects, clusters[i], min_pt, max_pt);

      // Project to 2D
      get2DProjectionFrom3D (min_pt, max_pt, cam_info_, data.roi[i]);

      //std::cerr << "COLUMNS x y z\nPOINTS 2\nDATA ascii" << std::endl;
      //std::cerr << min_pt[0] << " " << min_pt[1] << " " << min_pt[2] << std::endl;
      //std::cerr << max_pt[0] << " " << max_pt[1] << " " << max_pt[2] << std::endl;
      //pcl::io::savePCDFileBinary ("downsampled.pcd", all_minus_table);
      //pcl::io::savePCDFileBinary ("objects.pcd", cloud_objects);
      //pcl::io::savePCDFileBinary ("hull.pcd", table_hull);

      // Get the cluster points from the original point cloud
      std::vector<int> indices_in_box;
      pcl::getPointsInBox (*input_, min_pt, max_pt, indices_in_box);
      
      // Fill in the mask data
      for (size_t j = 0; j < indices_in_box.size (); ++j)
      {
        int ri = (int)(indices_in_box[j] / input_->width);
        int rc = (int)(indices_in_box[j] % input_->width);

        CV_IMAGE_ELEM (img_mask, uint8_t, ri, rc) = 255;
      }

      // Erode and Dilate
      //cvErode (img_mask, img_mask, NULL, 1);
      //cvDilate (img_mask, img_mask, NULL, 3);
      cvCopy (img_in, img_out, img_mask);
    }

    
    // Convert to ROS format
    img_bridge.fromIpltoRosImage (img_out, data.image_proc);
    cvReleaseImage (&img_out);
    cvReleaseImage (&img_mask);
    
    return true;
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Project 3D points to 2D image coordinates. Create a 2D reion of interest from a 3D bounding box.
    * \param min_pt the min 3d point (bounding box)
    * \param max_pt the max 3d point (bounding box)
    * \param cam_info the camera info message containing the camera parameters
    * \param roi the resultant 2D region of interest
    */
  void
    ExtractObjectRegions::get2DProjectionFrom3D (const Eigen3::Vector4f &min_pt, const Eigen3::Vector4f &max_pt,
                                                 const sensor_msgs::CameraInfoConstPtr &cam_info, sensor_msgs::RegionOfInterest &roi)
    {
      image_geometry::PinholeCameraModel camera_model;
      camera_model.fromCameraInfo (cam_info);

      Eigen3::MatrixXf points_3d = Eigen3::MatrixXf::Ones (4, 8);

      points_3d (0, 0) = min_pt[0]; points_3d (1, 0) = min_pt[1]; points_3d (2, 0) = min_pt[2]; 
      points_3d (0, 1) = max_pt[0]; points_3d (1, 1) = min_pt[1]; points_3d (2, 1) = min_pt[2]; 
      points_3d (0, 2) = min_pt[0]; points_3d (1, 2) = max_pt[1]; points_3d (2, 2) = min_pt[2]; 
      points_3d (0, 3) = max_pt[0]; points_3d (1, 3) = max_pt[1]; points_3d (2, 3) = min_pt[2]; 
      points_3d (0, 4) = min_pt[0]; points_3d (1, 4) = min_pt[1]; points_3d (2, 4) = max_pt[2]; 
      points_3d (0, 5) = max_pt[0]; points_3d (1, 5) = min_pt[1]; points_3d (2, 5) = max_pt[2]; 
      points_3d (0, 6) = min_pt[0]; points_3d (1, 6) = max_pt[1]; points_3d (2, 6) = max_pt[2]; 
      points_3d (0, 7) = max_pt[0]; points_3d (1, 7) = max_pt[1]; points_3d (2, 7) = max_pt[2]; 

      Eigen3::MatrixXf camera_pose = Eigen3::MatrixXf::Identity (3, 4);
      Eigen3::MatrixXf intrinsic_mat = Eigen3::MatrixXf::Identity (3, 3);
      intrinsic_mat (0, 0) = camera_model.fx ();
      intrinsic_mat (0, 2) = camera_model.cx ();
      intrinsic_mat (1, 1) = camera_model.fy ();
      intrinsic_mat (1, 2) = camera_model.cy ();

      Eigen3::MatrixXf points_2d = intrinsic_mat * camera_pose * points_3d;
      for (int i = 0; i < points_2d.cols (); ++i)
        points_2d.col (i) /= points_2d (2, i);
      
      float min_x = FLT_MAX, min_y = FLT_MAX, max_x = -FLT_MAX, max_y = -FLT_MAX;
      for (int i = 0; i < points_2d.cols (); ++i)
      {
        min_x = std::min (min_x, points_2d (0, i));
        min_y = std::min (min_y, points_2d (1, i));
        max_x = std::max (max_x, points_2d (0, i));
        max_y = std::max (max_y, points_2d (1, i));
      }
      roi.x_offset = (int)min_x;
      roi.y_offset = (int)min_y;
      roi.width  = (int)fabs (max_x - min_x);
      roi.height = (int)fabs (max_y - min_y);
    }


  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Project 3D points to 2D image coordinates. Create a 2D reion of interest from a 3D bounding box.
    * \param min_pt the min 3d point (bounding box)
    * \param max_pt the max 3d point (bounding box)
    * \param cam_info the camera info message containing the camera parameters
    * \param roi the resultant 2D region of interest
    * \param mask mask which is set at projected positions
    */
  void
    ExtractObjectRegions::get2DProjectionFrom3D (const Eigen3::Vector4f &min_pt, const Eigen3::Vector4f &max_pt,
                                                 const sensor_msgs::CameraInfoConstPtr &cam_info, sensor_msgs::RegionOfInterest &roi,
                                                 IplImage * mask)
    {
      image_geometry::PinholeCameraModel camera_model;
      camera_model.fromCameraInfo (cam_info);

      Eigen3::MatrixXf points_3d = Eigen3::MatrixXf::Ones (4, 8);

      points_3d (0, 0) = min_pt[0]; points_3d (1, 0) = min_pt[1]; points_3d (2, 0) = min_pt[2]; 
      points_3d (0, 1) = max_pt[0]; points_3d (1, 1) = min_pt[1]; points_3d (2, 1) = min_pt[2]; 
      points_3d (0, 2) = min_pt[0]; points_3d (1, 2) = max_pt[1]; points_3d (2, 2) = min_pt[2]; 
      points_3d (0, 3) = max_pt[0]; points_3d (1, 3) = max_pt[1]; points_3d (2, 3) = min_pt[2]; 
      points_3d (0, 4) = min_pt[0]; points_3d (1, 4) = min_pt[1]; points_3d (2, 4) = max_pt[2]; 
      points_3d (0, 5) = max_pt[0]; points_3d (1, 5) = min_pt[1]; points_3d (2, 5) = max_pt[2]; 
      points_3d (0, 6) = min_pt[0]; points_3d (1, 6) = max_pt[1]; points_3d (2, 6) = max_pt[2]; 
      points_3d (0, 7) = max_pt[0]; points_3d (1, 7) = max_pt[1]; points_3d (2, 7) = max_pt[2]; 

      Eigen3::MatrixXf camera_pose = Eigen3::MatrixXf::Identity (3, 4);
      Eigen3::MatrixXf intrinsic_mat = Eigen3::MatrixXf::Identity (3, 3);
      intrinsic_mat (0, 0) = camera_model.fx ();
      intrinsic_mat (0, 2) = camera_model.cx ();
      intrinsic_mat (1, 1) = camera_model.fy ();
      intrinsic_mat (1, 2) = camera_model.cy ();

      Eigen3::MatrixXf points_2d = intrinsic_mat * camera_pose * points_3d;
      for (int i = 0; i < points_2d.cols (); ++i)
        points_2d.col (i) /= points_2d (2, i);
      
      float min_x = FLT_MAX, min_y = FLT_MAX, max_x = -FLT_MAX, max_y = -FLT_MAX;
      for (int i = 0; i < points_2d.cols (); ++i)
      {
        min_x = std::min (min_x, points_2d (0, i));
        min_y = std::min (min_y, points_2d (1, i));
        max_x = std::max (max_x, points_2d (0, i));
        max_y = std::max (max_y, points_2d (1, i));
        
        const int px = static_cast<int> (points_2d (0, i));
        const int py = static_cast<int> (points_2d (1, i));
        //if (px >= 0 && px < mask->width && py >= 0 && py < mask->height)
        {
          CV_IMAGE_ELEM (mask, uint8_t, py, px) = 255;
        }
      }
      roi.x_offset = (int)min_x;
      roi.y_offset = (int)min_y;
      roi.width  = (int)fabs (max_x - min_x);
      roi.height = (int)fabs (max_y - min_y);
    }
    
    
    
};


    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Write point cloud data to a stream
      * \param fileStream the stream
      * \param cloud the point cloud data message
      */
    void
      ftd::writePointCloud (std::ofstream & fileStream, sensor_msgs::PointCloud2 &cloud)
    {
      if (cloud.data.size () == 0)
      {
        ROS_ERROR ("[writePointCloud] Input point cloud has no data!");
        return;
      }


      
      fileStream.write (reinterpret_cast<char*> (&cloud.height), sizeof (cloud.height));
      fileStream.write (reinterpret_cast<char*> (&cloud.width), sizeof (cloud.width));
      
      int fieldsSize = cloud.fields.size ();
      fileStream.write (reinterpret_cast<char*> (&fieldsSize), sizeof (fieldsSize));
      for (int fieldIndex = 0; fieldIndex < fieldsSize; ++fieldIndex)
      {
        int nameSize = cloud.fields[fieldIndex].name.size ();
        fileStream.write (reinterpret_cast<char*> (&nameSize), sizeof (nameSize));
        for (int stringIndex = 0; stringIndex < nameSize; ++stringIndex)
        {
          fileStream.write (reinterpret_cast<char*> (&cloud.fields[fieldIndex].name[stringIndex]), sizeof (cloud.fields[fieldIndex].name[stringIndex]));
        }
        
        fileStream.write (reinterpret_cast<char*> (&cloud.fields[fieldIndex].offset), sizeof (cloud.fields[fieldIndex].offset));
        fileStream.write (reinterpret_cast<char*> (&cloud.fields[fieldIndex].datatype), sizeof (cloud.fields[fieldIndex].datatype));
        fileStream.write (reinterpret_cast<char*> (&cloud.fields[fieldIndex].count), sizeof (cloud.fields[fieldIndex].count));
      }
      
      fileStream.write (reinterpret_cast<char*> (&cloud.is_bigendian), sizeof (cloud.is_bigendian));
      fileStream.write (reinterpret_cast<char*> (&cloud.point_step), sizeof (cloud.point_step));
      fileStream.write (reinterpret_cast<char*> (&cloud.row_step), sizeof (cloud.row_step));
      
      for (unsigned int dataIndex = 0; dataIndex < (cloud.row_step * cloud.height); ++dataIndex)
      {
        fileStream.write (reinterpret_cast<char*> (&cloud.data[dataIndex]), sizeof (cloud.data[dataIndex]));
      }
      
      fileStream.write (reinterpret_cast<char*> (&cloud.is_dense), sizeof (cloud.is_dense));


      return;
    }     
    
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Read point cloud data from a stream
      * \param fileStream the stream
      * \param cloud the point cloud data message
      */
    sensor_msgs::PointCloud2
      ftd::readPointCloud (std::ifstream & fileStream)
    {
      sensor_msgs::PointCloud2 cloud;

      

      fileStream.read (reinterpret_cast<char*> (&cloud.height), sizeof (cloud.height));
      fileStream.read (reinterpret_cast<char*> (&cloud.width), sizeof (cloud.width));
      
      int fieldsSize = 0;
      fileStream.read (reinterpret_cast<char*> (&fieldsSize), sizeof (fieldsSize));
      cloud.fields.resize (fieldsSize);
      for (int fieldIndex = 0; fieldIndex < fieldsSize; ++fieldIndex)
      {
        int nameSize = 0;
        fileStream.read (reinterpret_cast<char*> (&nameSize), sizeof (nameSize));
        for (int stringIndex = 0; stringIndex < nameSize; ++stringIndex)
        {
          char elem;
          fileStream.read (reinterpret_cast<char*> (&elem), sizeof (elem));
          cloud.fields[fieldIndex].name.push_back (elem);
        }
        
        fileStream.read (reinterpret_cast<char*> (&cloud.fields[fieldIndex].offset), sizeof (cloud.fields[fieldIndex].offset));
        fileStream.read (reinterpret_cast<char*> (&cloud.fields[fieldIndex].datatype), sizeof (cloud.fields[fieldIndex].datatype));
        fileStream.read (reinterpret_cast<char*> (&cloud.fields[fieldIndex].count), sizeof (cloud.fields[fieldIndex].count));
      }
      
      fileStream.read (reinterpret_cast<char*> (&cloud.is_bigendian), sizeof (cloud.is_bigendian));
      fileStream.read (reinterpret_cast<char*> (&cloud.point_step), sizeof (cloud.point_step));
      fileStream.read (reinterpret_cast<char*> (&cloud.row_step), sizeof (cloud.row_step));
      
      cloud.data.resize (cloud.row_step * cloud.height);
      for (unsigned int dataIndex = 0; dataIndex < (cloud.row_step * cloud.height); ++dataIndex)
      {
        fileStream.read (reinterpret_cast<char*> (&cloud.data[dataIndex]), sizeof (cloud.data[dataIndex]));
      }
      
      fileStream.read (reinterpret_cast<char*> (&cloud.is_dense), sizeof (cloud.is_dense));


      return cloud;
    }     

