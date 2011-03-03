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
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <boost/shared_ptr.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <pluginlib/class_list_macros.h>
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>
#include <pcl/ros/conversions.h>
#include <cv_bridge/CvBridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <rein/MaskArray.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace sensor_msgs;
using namespace rein;

class PCProjector : public nodelet::Nodelet
{
    typedef message_filters::Subscriber<sensor_msgs::PointCloud2> PointCloud2Subscriber;
    typedef message_filters::Subscriber<sensor_msgs::CameraInfo> CameraInfoSubscriber;
    typedef message_filters::TimeSynchronizer<CameraInfo, PointCloud2> SynchronizerCameraInfoPointCloud2;

    PointCloud2Subscriber point_cloud_sub_;
    CameraInfoSubscriber camera_info_sub_;
    SynchronizerCameraInfoPointCloud2 sync_sub_;
    ros::Publisher image_pub_;
    ros::Publisher masks_pub_;
    tf::TransformListener tf_;
    sensor_msgs::CvBridge bridge_;
public:

    PCProjector() : sync_sub_(103) {}

    void onInit()
    {
        ros::NodeHandle& nh = getMTPrivateNodeHandle();
        point_cloud_sub_.subscribe(nh,"point_cloud", 100);
        camera_info_sub_.subscribe(nh,"camera_info", 100);
        sync_sub_.connectInput(camera_info_sub_, point_cloud_sub_);
        sync_sub_.registerCallback(&PCProjector::dataCallback,this);
        image_pub_ = nh.advertise<Image>("image",1);
        masks_pub_ = nh.advertise<MaskArray>("masks",1);
        ROS_INFO("Nodelet init done");
    }


//    void refineMask(cv::Mat& mask, const cv::Rect bbox)

//    }


    void dataCallback(const CameraInfoConstPtr& camera_info, 
                    const PointCloud2ConstPtr& point_cloud)
    {
        bool found_transform = tf_.waitForTransform(camera_info->header.frame_id, point_cloud->header.frame_id,
                ros::Time::now(), ros::Duration(1.0));

        ROS_ASSERT_MSG(found_transform, "Could not transform to camera frame");
        tf::StampedTransform transform;

        tf_.lookupTransform(camera_info->header.frame_id, point_cloud->header.frame_id, camera_info->header.stamp, transform);
        PointCloud2 transformed_cloud;
        pcl_ros::transformPointCloud(camera_info->header.frame_id, transform, *point_cloud, transformed_cloud);

        // Initialize camera model
        image_geometry::PinholeCameraModel cam_model;
        cam_model.fromCameraInfo(*camera_info);
        pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
        pcl::fromROSMsg(transformed_cloud, pcl_cloud);

        cv::Mat mask(camera_info->height, camera_info->width, CV_8UC1);
        mask = cv::Scalar::all(0);


        int xmin=-1,xmax=-1,ymin=-1,ymax=-1;
        BOOST_FOREACH (const pcl::PointXYZ& pt, pcl_cloud.points) {
            cv::Point3d cv_pt(pt.x, pt.y, pt.z);
            cv::Point2d uv;

            cam_model.project3dToPixel(cv_pt, uv);

            if (uv.x>=0 && uv.x<camera_info->width && uv.y>=0 && uv.y<camera_info->height) {
                mask.at<uchar>(uv.y,uv.x) = 255;
                if (xmin<0 || xmin>uv.x) xmin = uv.x;
                if (xmax<0 || xmax<uv.x) xmax = uv.x;
                if (ymin<0 || ymin>uv.y) ymin = uv.y;
                if (ymax<0 || ymax<uv.y) ymax = uv.y;
            }
        }

        cv::dilate(mask,mask,cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(5,5)));
        cv::erode(mask,mask,cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(5,5)));


		for (int y=0;y<mask.rows;++y) {
			for (int x =0; x<mask.cols;++x) {
				if (mask.at<uchar>(y,x)>0) {
	                if (xmin<0 || xmin>x) xmin = x;
	                if (xmax<0 || xmax<x) xmax = x;
	                if (ymin<0 || ymin>y) ymin = y;
	                if (ymax<0 || ymax<y) ymax = y;
				}
			}
		}


//		printf("%d %d %d %d\n", xmin, xmax, ymin, ymax);
        MaskArray masks;
        masks.masks.resize(1);
        Rect& r = masks.masks[0].roi;
        r.x = xmin;
        r.y = ymin;
        r.width = xmax-xmin;
        r.height = ymax-ymin;
        cv::Rect roi;
        roi.x = r.x; roi.y = r.y; roi.width = r.width; roi.height = r.height;

//        refineMask(mask, roi);


        cv::Mat small_mask;
        mask(roi).copyTo(small_mask);

        IplImage ipl_small_mask = (IplImage) small_mask;
        masks.masks[0].mask = *bridge_.cvToImgMsg(&ipl_small_mask);

        IplImage ipl_mask = (IplImage)mask;
        ImagePtr im_ptr = bridge_.cvToImgMsg(&ipl_mask);
        im_ptr->header = camera_info->header;
        image_pub_.publish(im_ptr);

        masks.header = camera_info->header;
        masks_pub_.publish(masks);

        
    }


};

/**
 * Pluginlib declaration. This is needed for the nodelet to be dynamically loaded/unloaded
 */
PLUGINLIB_DECLARE_CLASS (rein, PCProjector, PCProjector, nodelet::Nodelet);
