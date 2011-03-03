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


#include <nodelet/nodelet.h>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pluginlib/class_list_macros.h>
#include <rein/MaskArray.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>



using namespace sensor_msgs;
using namespace rein;

class PointCloudResync : public nodelet::Nodelet
{
    typedef message_filters::Subscriber<sensor_msgs::Image> ImageSubscriber;
    typedef message_filters::Subscriber<sensor_msgs::CameraInfo> CameraInfoSubscriber;
    typedef message_filters::Subscriber<sensor_msgs::PointCloud2> PointCloud2Subscriber;
    typedef message_filters::sync_policies::ApproximateTime<Image, CameraInfo, PointCloud2> ApproxSyncPolicy;
    typedef message_filters::Synchronizer<ApproxSyncPolicy> Synchronizer;

    ImageSubscriber image_sub_;
    CameraInfoSubscriber cam_info_sub_;
    PointCloud2Subscriber pc_sub_;
    Synchronizer sync_sub_;
    ros::Publisher image_pub_;
    ros::Publisher cam_info_pub_;
    ros::Publisher pc_pub_;
public:

    PointCloudResync() : sync_sub_(103) {}

    void onInit()
    {
        ros::NodeHandle& nh = getMTPrivateNodeHandle();
        image_sub_.subscribe(nh,"image", 100);
        cam_info_sub_.subscribe(nh,"camera_info", 100);
        pc_sub_.subscribe(nh,"point_cloud", 100);
        sync_sub_.connectInput(image_sub_,cam_info_sub_, pc_sub_);
        sync_sub_.registerCallback(&PointCloudResync::dataCallback,this);
        image_pub_ = nh.advertise<Image>("image_out",1);
        cam_info_pub_ = nh.advertise<CameraInfo>("camera_info_out",1);
        pc_pub_ = nh.advertise<PointCloud2>("point_cloud_out",1);
        ROS_INFO("Nodelet init done");
    }


    void dataCallback(const ImageConstPtr& image,
                    const CameraInfoConstPtr& cam_info, 
                    const PointCloud2ConstPtr& point_cloud)
    {
        PointCloud2 pc_new = *point_cloud;
        pc_new.header = cam_info->header;
        pc_pub_.publish(pc_new);
        image_pub_.publish(image);
        cam_info_pub_.publish(cam_info);
    }


};


/**
 * Pluginlib declaration. This is needed for the nodelet to be dynamically loaded/unloaded
 */
PLUGINLIB_DECLARE_CLASS (rein, PointCloudResync, PointCloudResync, nodelet::Nodelet);
