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
#include <cv_bridge/CvBridge.h>
#include <opencv2/highgui/highgui.hpp>

using namespace sensor_msgs;
using namespace rein;

class GrabCut : public nodelet::Nodelet
{
    typedef message_filters::Subscriber<sensor_msgs::Image> ImageSubscriber;
    typedef message_filters::Subscriber<rein::MaskArray> MaskArraySubscriber;
    typedef message_filters::TimeSynchronizer<Image,MaskArray> SynchronizerImageMaskArray;

    ImageSubscriber image_sub_;
    MaskArraySubscriber masks_sub_;
    SynchronizerImageMaskArray sync_sub_;
    ros::Publisher masks_pub_;
    ros::Publisher patch_pub_;
    sensor_msgs::CvBridge bridge_;
public:

    GrabCut() : sync_sub_(103) {}

    void onInit()
    {
        ros::NodeHandle& nh = getMTPrivateNodeHandle();
        image_sub_.subscribe(nh,"image", 100);
        masks_sub_.subscribe(nh,"masks", 100);
        sync_sub_.connectInput(image_sub_, masks_sub_);
        sync_sub_.registerCallback(&GrabCut::dataCallback,this);
        masks_pub_ = nh.advertise<MaskArray>("masks_out",1);
        patch_pub_ = nh.advertise<Image>("image_mask",1);
    }


    void refine_mask(const cv::Mat& image, cv::Rect& bbox, cv::Mat& mask)
    {
//    	cv::dilate(mask,mask,cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(2,2)));
//    	cv::erode(mask,mask,cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(2,2)));


    	cv::Mat gc_image(image.size(), CV_8UC3);
    	cv::cvtColor(image,gc_image, CV_GRAY2RGB);

    	cv::Mat gc_mask(mask.size(),CV_8UC1, cv::Scalar(cv::GC_BGD));

    	cv::Mat tmp_mask;
    	cv::dilate(mask,tmp_mask,cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(50,50)));
    	gc_mask.setTo(cv::Scalar(cv::GC_PR_BGD), tmp_mask);

    	cv::dilate(mask,tmp_mask,cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(10,10)));
    	gc_mask.setTo(cv::Scalar(cv::GC_PR_FGD), tmp_mask);
    	cv::erode(mask,tmp_mask,cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(10,10)));
    	gc_mask.setTo(cv::Scalar(cv::GC_FGD), mask);

//		cv::namedWindow("gc_mask",1);
//		cv::imshow("gc_mask", gc_mask*70);
//		cv::waitKey(40);

		cv::Mat bgdModel;
		cv::Mat fgdModel;
		cv::grabCut(gc_image, gc_mask, bbox, bgdModel, fgdModel, 1, cv::GC_INIT_WITH_MASK);

		mask = gc_mask & 1;

		cv::namedWindow("mask",1);
		cv::imshow("mask", mask*100);
		cv::waitKey(40);


		int xmin = -1, xmax = -1, ymin = -1, ymax = -1;

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
		bbox.x = xmin;
		bbox.y = ymin;
		bbox.width = xmax-xmin;
		bbox.height = ymax-ymin;
    }


    void dataCallback(const ImageConstPtr& image, 
                    const MaskArrayConstPtr& masks)
    {
    	static CvBridge bridge;

    	cv::Mat cv_image = bridge_.imgMsgToCv(image);
        MaskArray masks_out = *masks;
    	for (size_t i=0;i<masks->masks.size();++i) {
    		const Rect& r = masks->masks[i].roi;
    		bridge.fromImage(masks->masks[i].mask);
    		cv::Rect bbox(r.x,r.y,r.width, r.height);
    		patch_pub_.publish(masks->masks[i].mask);
    		cv::Mat mask(bridge.toIpl(),false);
    		cv::Mat img_mask(cv_image.size(), CV_8UC1);
    		img_mask = cv::Scalar::all(0);
    		cv::Mat tmp = img_mask(bbox);
    		mask.copyTo(tmp);

    		refine_mask(cv_image, bbox, img_mask);
    		img_mask(bbox).copyTo(mask);
    		IplImage ipl_mask = (IplImage)mask;
    		masks_out.masks[i].mask = *bridge_.cvToImgMsg(&ipl_mask);
    		masks_out.masks[i].roi.x  = bbox.x;
    		masks_out.masks[i].roi.y  = bbox.y;
    		masks_out.masks[i].roi.width  = bbox.width;
    		masks_out.masks[i].roi.height  = bbox.height;
    	}
        masks_pub_.publish(masks_out);
    }


};


/**
 * Pluginlib declaration. This is needed for the nodelet to be dynamically loaded/unloaded
 */
PLUGINLIB_DECLARE_CLASS (rein, GrabCut, GrabCut, nodelet::Nodelet);
