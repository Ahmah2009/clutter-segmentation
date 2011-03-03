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
 * $Id: compute_normals.cpp 29038 2010-04-23 16:44:44Z rusu $
 *
 */

#ifndef PCL_COMPUTE_NORMALS_H_
#define PCL_COMPUTE_NORMALS_H_

#include "pcl/point_types.h"

#include "pcl/features/normal_3d.h"
#include "pcl/kdtree/kdtree.h"
#include "pcl/kdtree/organized_data.h"

class ComputeNormals
{
//  typedef pcl::PointXYZRGB Point;
  typedef pcl::PointXYZ Point;
  typedef pcl::OrganizedDataIndex<Point>::Ptr KdTreePtr;

  public:
    // PCL objects
    KdTreePtr normals_tree_;
    pcl::NormalEstimation<Point, pcl::Normal> n3d_;

    pcl::PointCloud<Point>::ConstPtr cloud_;
    pcl::PointCloud<pcl::Normal>::ConstPtr cloud_normals_;
    
    int k_;
    
    ComputeNormals () : k_ (50) // 10 k-neighbors by default
    {
      normals_tree_ = boost::make_shared<pcl::OrganizedDataIndex<Point> > ();
      normals_tree_->setSearchWindowAsK (k_);
      normals_tree_->setMaxDistance (0.1);

      // Normal estimation parameters
      n3d_.setKSearch (k_);
      //n3d_.setRadiusSearch (0.015);
      n3d_.setSearchMethod (normals_tree_);
    }

    inline void
      setKSearch (int k)
    {
      k_ = k;
      normals_tree_->setSearchWindowAsK (k_);
      // Normal estimation parameters
      n3d_.setKSearch (k_);
      //n3d_.setRadiusSearch (0.015);
      n3d_.setSearchMethod (normals_tree_);
    }
    
    void
      setInputCloud (const sensor_msgs::PointCloud2ConstPtr &cloud2)
    {
      pcl::PointCloud<Point> cloud;
      pcl::fromROSMsg (*cloud2, cloud);
      cloud_ = boost::make_shared<const pcl::PointCloud<Point> > (cloud);
    }

    void
      compute ()
    {
      // ---[ Estimate the point normals
      pcl::PointCloud<pcl::Normal> cloud_normals;
      n3d_.setInputCloud (cloud_);
      n3d_.compute (cloud_normals);
      cloud_normals_.reset (new pcl::PointCloud<pcl::Normal> (cloud_normals));
      ROS_INFO ("%d normals estimated.", (int)cloud_normals.points.size ());
    }

    void
      getOutput (pcl::PointCloud<pcl::Normal> &output)
    {
      output = *cloud_normals_;
    }
};

class ComputeNormalsUnorganized
{
  typedef pcl::PointXYZ Point;
  typedef pcl::KdTreeANN<Point>::Ptr KdTreePtr;

  public:
    // PCL objects
    KdTreePtr normals_tree_;
    pcl::NormalEstimation<Point, pcl::Normal> n3d_;

    pcl::PointCloud<Point>::ConstPtr cloud_;
    pcl::PointCloud<pcl::Normal>::ConstPtr cloud_normals_;
    
    int k_;
    
    ComputeNormalsUnorganized () : k_ (0)
    {
      normals_tree_ = boost::make_shared<pcl::KdTreeANN<Point> > ();
      // Normal estimation parameters
      //n3d_.setKSearch (k_);
      n3d_.setRadiusSearch (0.015);
      n3d_.setSearchMethod (normals_tree_);
    }

    void
      setInputCloud (const sensor_msgs::PointCloud2ConstPtr &cloud2)
    {
      pcl::PointCloud<Point> cloud;
      pcl::fromROSMsg (*cloud2, cloud);
      cloud_ = boost::make_shared<const pcl::PointCloud<Point> > (cloud);
    }

    void
      compute ()
    {
      // ---[ Estimate the point normals
      pcl::PointCloud<pcl::Normal> cloud_normals;
      n3d_.setInputCloud (cloud_);
      n3d_.compute (cloud_normals);
      cloud_normals_.reset (new pcl::PointCloud<pcl::Normal> (cloud_normals));
      ROS_INFO ("%d normals estimated.", (int)cloud_normals.points.size ());
    }

    void
      getOutput (pcl::PointCloud<pcl::Normal> &output)
    {
      output = *cloud_normals_;
    }
};

#endif // PCL_COMPUTE_NORMALS_H_


