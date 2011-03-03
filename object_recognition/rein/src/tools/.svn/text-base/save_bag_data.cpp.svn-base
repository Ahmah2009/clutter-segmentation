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
#include <pcl/io/pcd_io.h>
#include <boost/lexical_cast.hpp>
#include <ros/serialization.h>
#include <rein/nodelets/type_conversions.h>

using namespace sensor_msgs;
using namespace rein;

namespace ser = ros::serialization;



template <typename M>
void serialize(const std::string filename, const M& msg)
{
	FILE* f = fopen(filename.c_str(), "w");
	size_t length = ser::serializationLength(msg);
	uint8_t* buff = new uint8_t[length];
	ser::OStream stream(buff,length);
	ser::serialize(stream,msg);
	fwrite(buff, length, 1, f);
	fclose(f);
	delete[] buff;
}

class SaveData : public nodelet::Nodelet
{
    typedef message_filters::Subscriber<sensor_msgs::PointCloud2> PointCloud2Subscriber;
    typedef message_filters::Subscriber<sensor_msgs::Image> ImageSubscriber;
    typedef message_filters::Subscriber<rein::MaskArray> MaskArraySubscriber;
    typedef message_filters::TimeSynchronizer<Image, PointCloud2, rein::MaskArray> Synchronizer;

    PointCloud2Subscriber point_cloud_sub_;
    ImageSubscriber image_sub_;
    MaskArraySubscriber masks_sub_;
    Synchronizer sync_sub_;
public:

    SaveData() : sync_sub_(103) {}

    void onInit()
    {
        ros::NodeHandle& nh = getMTPrivateNodeHandle();
        point_cloud_sub_.subscribe(nh,"point_cloud", 100);
        image_sub_.subscribe(nh,"image", 100);
        masks_sub_.subscribe(nh,"masks", 100);
        sync_sub_.connectInput(image_sub_, point_cloud_sub_, masks_sub_);
        sync_sub_.registerCallback(&SaveData::dataCallback,this);
        ROS_INFO("Nodelet init done");
    }


//    void refineMask(cv::Mat& mask, const cv::Rect bbox)

//    }


    void dataCallback(const ImageConstPtr& image,
                    const PointCloud2ConstPtr& point_cloud,
                    const MaskArrayConstPtr& masks)
    {
    	static CvBridge bridge;
    	static CvBridge bridge2;
    	static int index = 0;
    	char name[100];

    	if (masks->masks.size()==0) return;
    	const rein::Rect& r = masks->masks[0].roi;
    	if (r.width+r.height<150) {
    		return;
    	}

    	printf("Saving data\n");

    	sprintf(name, "%.4d_cloud.pcd", index);
      pcl::PCDWriter w; w.writeBinary (name, *point_cloud);
    	sprintf(name, "%.4d_cloud.msg", index);
    	serialize(name, *point_cloud);

    	sprintf(name, "%.4d_image.png", index);
    	cv::Mat cv_image = bridge.imgMsgToCv(image);
    	cv::imwrite(name, cv_image);

    	cv::Mat ann_img = cv::Mat(cv_image.rows, cv_image.cols, CV_8UC3);
    	cv::cvtColor(cv_image, ann_img, CV_GRAY2RGB);

    	sprintf(name, "%.4d_bbox.txt", index);
    	FILE* f = fopen(name, "w");
    	for (size_t i=0;i<masks->masks.size();++i) {
    		const rein::Mask& m = masks->masks[i];
    		fprintf(f, "%d %d %d %d\n", m.roi.x, m.roi.y, m.roi.width, m.roi.height);
        	cv::rectangle(ann_img, cv::Point(m.roi.x,m.roi.y),
        			cv::Point(m.roi.x+m.roi.width,m.roi.y+m.roi.height), cv::Scalar(0,0,255),2);

//        	sensor_msgs::Image msk = m.mask;
//        	cv::Rect roi(m.roi.x, m.roi.y, m.roi.width, m.roi.height);
//        	cv::Mat cv_mask = bridge2.imgMsgToCv(boost::make_shared<sensor_msgs::Image>(msk));
//        	cv::Mat im_roi = ann_img(roi);
//        	cv::add(im_roi,cv::Scalar(80,0,0),im_roi,cv_mask);
//        	cv::namedWindow("mask",1);
//        	cv::imshow("mask",cv_mask);
//        	cv::waitKey(30);


    	}
    	fclose(f);

    	sprintf(name, "%.4d_image_annotated.png", index);
    	cv::imwrite(name, ann_img);

    	sprintf(name, "%.4d_masks.msg", index);
    	serialize(name, *masks);



    	index++;
    }


};

/**
 * Pluginlib declaration. This is needed for the nodelet to be dynamically loaded/unloaded
 */
PLUGINLIB_DECLARE_CLASS (rein, SaveData, SaveData, nodelet::Nodelet);
