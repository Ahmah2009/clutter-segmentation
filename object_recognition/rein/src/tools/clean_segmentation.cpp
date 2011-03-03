/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
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
 * $Id$
 *
 */

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <sensor_msgs/PointCloud2.h>
#include <boost/shared_ptr.hpp>
#include <pluginlib/class_list_macros.h>
#include <pcl/ros/conversions.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/voxel_grid.h>


using namespace sensor_msgs;

class CleanSegmentation : public nodelet::Nodelet
{
    ros::Subscriber point_cloud_sub_;
    ros::Publisher point_cloud_pub_;
public:

    CleanSegmentation() {}

    void onInit()
    {
        ros::NodeHandle& nh = getMTPrivateNodeHandle();
        point_cloud_sub_ = nh.subscribe("point_cloud", 100, &CleanSegmentation::dataCallback, this);
        point_cloud_pub_ = nh.advertise<PointCloud2>("point_cloud_out",1);
        ROS_INFO("Nodelet init done");
    }

    void dataCallback(const PointCloud2::ConstPtr& point_cloud)
    {
        typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
        typedef pcl::VoxelGrid<pcl::PointXYZ> VoxelGrid;
        typedef pcl::EuclideanClusterExtraction<pcl::PointXYZ> EuclideanClusterExtraction;

        EuclideanClusterExtraction clusterer;
//        VoxelGrid grid;

    	PointCloud::Ptr cloud = boost::make_shared<PointCloud>();
    	pcl::fromROSMsg(*point_cloud, *cloud);

//    	grid.setLeafSize(0.001,0.001,0.001);
//    	grid.setInputCloud(cloud);
//    	PointCloud::Ptr cloud_downsampled = boost::make_shared<PointCloud>();
//    	grid.filter(*cloud_downsampled);


        clusterer.setClusterTolerance (0.01);
        clusterer.setMinClusterSize (100);
        clusterer.setInputCloud(cloud);
        std::vector<pcl::PointIndices> clusters;
        clusterer.extract(clusters);

        if (clusters.size()==0) return;

        printf("Found %d clusters\n", (int)clusters.size());
        size_t max_size = clusters[0].indices.size();
        int max_idx = 0;
        for (size_t i=0;i<clusters.size();++i) {
        	printf("Cluster %d, size: %d\n", int(i), int(clusters[i].indices.size()));
        	if (clusters[i].indices.size()>max_size) {
        		max_size = clusters[i].indices.size();
        		max_idx = i;
        	}
        }


    	PointCloud::Ptr object_cloud = boost::make_shared<PointCloud>();
    	copyPointCloud (*cloud, clusters[max_idx], *object_cloud);
        
    	sensor_msgs::PointCloud2 cloud_msg;
    	pcl::toROSMsg(*object_cloud,cloud_msg);
    	point_cloud_pub_.publish(cloud_msg);
    }


};

/**
 * Pluginlib declaration. This is needed for the nodelet to be dynamically loaded/unloaded
 */
PLUGINLIB_DECLARE_CLASS (rein, CleanSegmentation, CleanSegmentation, nodelet::Nodelet);
