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
 * $Id$
 * Author: Ethan Rublee
 * run with:
 rosrun tod_stub image_pcl_recorder \
 camera:=/camera/rgb points2:=/camera/depth/points2 time_interval:=0.5
 */
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <boost/shared_ptr.hpp>
#include <rosbag/bag.h>

#include <pcl/io/io.h>
using std::string;
using namespace sensor_msgs;

namespace tod_stub
{

class ImagePclRecorder
{
  typedef message_filters::Subscriber<sensor_msgs::PointCloud2> PointCloud2Subscriber;
  typedef message_filters::Subscriber<sensor_msgs::CameraInfo> CameraInfoSubscriber;
  typedef message_filters::Subscriber<sensor_msgs::Image> ImageSubscriber;
  typedef message_filters::sync_policies::ApproximateTime<Image, CameraInfo, PointCloud2> ApproxSyncPolicy;
  typedef message_filters::Synchronizer<ApproxSyncPolicy> SynchronizerCameraInfoPointCloud2;

  ImageSubscriber image_sub_;
  PointCloud2Subscriber point_cloud_sub_;
  CameraInfoSubscriber camera_info_sub_;
  SynchronizerCameraInfoPointCloud2 sync_sub_;
  ros::NodeHandle nh_;

  string bag_filename_;
  string camera_topic_;
  string points2_topic_;

  ros::Time prev_;
  double time_interval_;
  rosbag::Bag bag_;
public:

  ImagePclRecorder() :
    sync_sub_(15), time_interval_(0.001)
  {
    onInit();
  }

  void onInit()
  {
    ros::NodeHandle& nh = nh_;

    //get the bag filename that will be used for saving
    //messages to
    if (!nh.getParam("bag", bag_filename_)){
      ROS_ERROR("bag filename not set");
      nh.shutdown();
      return;
    }

    nh.getParam("interval", time_interval_);
    if (time_interval_ < 0.001)
    {
      time_interval_ = 0.001;
    }
    prev_ = ros::Time::now();

    //may be remapped to : /camera/rgb
    camera_topic_ = nh.resolveName("camera", true);

    //should be remapped to something like: /camera/depth/points2
    points2_topic_ = nh.resolveName("points2", true);

    //subscribe to topics
    point_cloud_sub_.subscribe(nh, points2_topic_, 15);
    image_sub_.subscribe(nh, camera_topic_ + "/image_color", 15);
    camera_info_sub_.subscribe(nh, camera_topic_ + "/camera_info", 15);

    sync_sub_.connectInput(image_sub_, camera_info_sub_, point_cloud_sub_);
    sync_sub_.registerCallback(&ImagePclRecorder::dataCallback, this);
    bag_.open(bag_filename_, rosbag::bagmode::Write);
    //bag_.close();

    ROS_INFO_STREAM("camera topic is set to " << camera_topic_);
    ROS_INFO_STREAM("bag will be recorded to " << bag_filename_);
    ROS_INFO_STREAM("time interval set to attempt" << 1 / time_interval_ << " captures per second.");
    ROS_INFO("init done");
  }

  void dataCallback(const ImageConstPtr& image, const CameraInfoConstPtr& camera_info,
                    const PointCloud2ConstPtr& point_cloud)
  {
    ros::Time n = ros::Time::now();
    if ((n - prev_).sec > time_interval_)
    {
      prev_ = n;
      ROS_INFO("saving point cloud and image");
      bag_.write("image", n, image);
      bag_.write("camera_info", n, camera_info);
      bag_.write("points2", n, point_cloud);
    }
  }
};

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_pcl_recorder");
  tod_stub::ImagePclRecorder imagePclRecorder;
  ros::spin();
  return 0;
}

