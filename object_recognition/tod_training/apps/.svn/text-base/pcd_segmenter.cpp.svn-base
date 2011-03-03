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
 * $Id: surface_convex_hull.cpp 34612 2010-12-08 01:06:27Z rusu $
 *
 */

/**
 \author Radu Bogdan Rusu

 @b surface_convex_hull exemplifies how to use a ConvexHull surface reconstruction.
 **/
#include "pcl/ModelCoefficients.h"

#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"

#include "pcl/sample_consensus/method_types.h"
#include "pcl/sample_consensus/model_types.h"
#include "pcl/filters/passthrough.h"
#include "pcl/filters/project_inliers.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "pcl/surface/convex_hull.h"

#include "pcl/segmentation/extract_polygonal_prism_data.h"
#include "pcl/filters/extract_indices.h"
#include "pcl/registration/registration.h"

#include "tod/core/PoseRT.h"
#include "tod/core/Camera.h"
#include "tod/training/clouds.h"
#include <tod/core/Features3d.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/eigen.hpp>
/* ---[ */
int main(int argc, char** argv)
{
  if (argc != 5)
  {
    std::cout << "usage:\n" << argv[0] << " cloud.pcd pose.yaml camera.yml out.pcd" << std::endl;
    return 1;
  }

  tod::Camera camera(argv[3], tod::Camera::TOD_YAML);

  pcl::PointCloud<pcl::PointXYZRGB> cloud, cloud_filtered, cloud_projected;
  pcl::PCDReader reader;
  reader.read(argv[1], cloud);
  tod::PoseRT pose;
  cv::Mat _R;

  cv::FileStorage fs(argv[2], cv::FileStorage::READ);
  pose.read(fs["pose"]);

  cv::Rodrigues(pose.rvec,_R);
  Eigen::Matrix<float,3,3> R;
  Eigen::Vector3f T;
  cv::cv2eigen(pose.tvec,T);
  cv::cv2eigen(_R,R);



  // Build a filter to remove spurious NaNs
  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud(boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB> >(cloud));
  float box = 0.1;
  pass.setFilterFieldName("z");
  pass.setFilterLimits(T[2] - box, T[2] + box);
  pass.filter(cloud_filtered);

  pass.setInputCloud(boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB> >(cloud_filtered));
  pass.setFilterFieldName("x");
  pass.setFilterLimits(T[0] - box, T[0]  + box);
  pass.filter(cloud_filtered);

  pass.setInputCloud(boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB> >(cloud_filtered));
  pass.setFilterFieldName("y");
  pass.setFilterLimits(T[1] - box, T[1]  + box);
  pass.filter(cloud_filtered);
  ROS_INFO ("PointCloud after filtering has: %d data points.", (int)cloud_filtered.points.size ());

  pcl::ModelCoefficients coefficients;
  cv::Vec4f plane = pose.toPlanarCoefficients();
  coefficients.values.resize(4);
  coefficients.values[0] = plane[0];
  coefficients.values[1] = plane[1];
  coefficients.values[2] = plane[2];
  coefficients.values[3] = plane[3];

//  pcl::PointIndices inliers;
//  // Create the segmentation object
//  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
//  // Optional
//  seg.setOptimizeCoefficients(false);
//  // Mandatory
//  seg.setModelType(pcl::SACMODEL_PLANE);
//  seg.setMethodType(pcl::SAC_RANSAC);
//  seg.setDistanceThreshold(0.01);
//
//  seg.setInputCloud(boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB> >(cloud_filtered));
//  seg.segment(inliers, coefficients);

  // Create the filtering object
//  pcl::ExtractIndices<pcl::PointXYZRGB> extract;
//  // Extract the inliers
//  extract.setInputCloud(boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB> >(cloud_filtered));
//  extract.setIndices(boost::make_shared<pcl::PointIndices>(inliers));
//  extract.setNegative(false);
//  extract.filter(cloud_projected);

  pcl::PointCloud<pcl::PointXYZRGB> board,board_transformed;

    // Fill in the cloud data
  board.width  = 4;
  board.height = 1;
  board.points.resize (cloud.width * cloud.height);

  board.points[0].x=-0.1f, board.points[0].y=-0.1f, board.points[0].z = 0.0f;
  board.points[1].x=-0.1f, board.points[1].y=+0.1f, board.points[1].z = 0.0f;
  board.points[2].x=+0.1f, board.points[2].y=+0.1f, board.points[2].z = 0.0f;
  board.points[3].x=+0.1f, board.points[3].y=-0.1f, board.points[3].z = 0.0f;

  std::cout << "R = " << R << "\n";
  std::cout << "T = " << T <<std::endl;;

  Eigen::Quaternionf q(R);
  std::cout << "q = " << q.vec() << "\n";

  pcl::transformPointCloud(board,board,T, q);



  // Project the model inliers
  pcl::ProjectInliers<pcl::PointXYZRGB> proj;
  proj.setModelType(pcl::SACMODEL_PLANE);
  proj.setInputCloud(boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB> >(cloud_filtered));
  proj.setModelCoefficients(boost::make_shared<pcl::ModelCoefficients>(coefficients));
  proj.filter(cloud_projected);

  // Create a Convex Hull representation of the projected inliers
  pcl::PointCloud<pcl::PointXYZRGB> cloud_hull;
  pcl::ConvexHull<pcl::PointXYZRGB> chull;
  chull.setInputCloud(boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB> >(cloud_projected));
  chull.reconstruct(cloud_hull);


  ROS_INFO ("Convex hull has: %d data points.", (int)cloud_hull.points.size ());

  pcl::ExtractPolygonalPrismData<pcl::PointXYZRGB> prism;

  // Consider objects starting at 1cm from the table and ending at 0.5m
  float object_min_height = -0.01;
  float object_max_height = 0.5;

  prism.setHeightLimits(object_min_height, object_max_height);

  // ---[ Get the objects on top of the table
  pcl::PointIndices cloud_object_indices;
  prism.setInputCloud(boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB> >(cloud_filtered));
  prism.setInputPlanarHull(boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB> >(cloud_hull));
  prism.segment(cloud_object_indices);
  std::cout << "[TableObjectDetector::input_callback] Number of object point indices: "
      << cloud_object_indices.indices.size() << std::endl;

  pcl::PointCloud<pcl::PointXYZRGB> cloud_objects;
  pcl::ExtractIndices<pcl::PointXYZRGB> extract_object_indices;
  extract_object_indices.setInputCloud(boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB> >(cloud_filtered));
  extract_object_indices.setIndices(boost::make_shared<const pcl::PointIndices>(cloud_object_indices));
  extract_object_indices.filter(cloud_objects);
  std::cout << "[TableObjectDetector::input_callback] Number of object point candidates: "
      << (int)cloud_objects.points.size() << std::endl;

  if (cloud_objects.points.size() == 0)
    return 1;

  pcl::PCDWriter writer;
  writer.write(argv[4], cloud_objects, true);

  std::vector<cv::Point3f> points;
  tod::PCLToPoints(points,cloud_objects);

  cv::Mat mask(camera.image_size,CV_8UC1);

  tod::drawProjectedPoints(camera,points,mask);

  cv::imshow("mask", mask);
  cv::imwrite("mask.png",mask);
  cv::waitKey();



  return (0);
}

