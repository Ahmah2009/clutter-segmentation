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

/**
\author Marius Muja
 **/


#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/CvBridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <sstream>

#include "rein/DetectionArray.h"


#include <execinfo.h>
#include <signal.h>
#include <stdlib.h>

template <typename T>
std::string tostr(const T& t)
{
	std::ostringstream os;
	os<<t;
	return os.str();
}



void baz() {
	int *foo = (int*)-1; // make a bad pointer
	printf("%d\n", *foo);       // causes segfault
}



class DetectionDisplayer {
public:

	DetectionDisplayer() : it_(nh_), sync_sub_(100)
	{
		image_sub_.subscribe(it_,"image",1);
		detections_sub_.subscribe(nh_,"detections",1);

		sync_sub_.connectInput(image_sub_,detections_sub_);
		sync_sub_.registerCallback(&DetectionDisplayer::detectionCallback, this);

		cv::namedWindow("Image",1);
	}


	void detectionCallback(const sensor_msgs::ImageConstPtr& img,
			const rein::DetectionArrayConstPtr& detections)
	{
		cv::Mat image = bridge_.imgMsgToCv(img, "bgr8");
		int wmax = image.cols;
		int hmax = image.rows;
		static int frame_idx = 0;

		const std::vector<rein::Detection>& detect = detections->detections;
//		ROS_INFO("Detect size=%d\n",(int)detect.size());
		for (size_t i=0;i<detect.size();++i) {

			int x = detect[i].mask.roi.x;
			int y = detect[i].mask.roi.y;
			int w = detect[i].mask.roi.width;
			int h = detect[i].mask.roi.height;

			int bl = (2*2555)%200;
			int gr = (2*433)%224;
			int rd = (2*2020)%210;

			cv::rectangle(image,cv::Point(x,y),cv::Point(x+w, y+h),cv::Scalar(bl,gr,rd),2);

			sensor_msgs::CvBridge bridge;
			bridge.fromImage(detect[i].mask.mask);
			cv::Mat mask = bridge.toIpl();
//			if(x+w > wmax) ROS_INFO("x(%d): x+w=%d > wmax=%d\n",i,x+w,wmax);
//			if(y+h > hmax) ROS_INFO("y(%d): y+h=%d > hmax=%d\n",i,y+h,hmax);

			if (mask.data && x>=0 && y>=0) {
//				ROS_INFO("%d: (x,y,w,h) (%d, %d, %d, %d) vs Iwh(%d, %d) vs M(%d, %d)",(int)i,x,y,w,h,image.cols,image.rows,mask.cols,mask.rows);
				if(x+w >= wmax) w = wmax - x - 1; //Keep ROI in bounds
				if(y+h >= hmax) h = hmax - y - 1;
				cv::Rect roi(x,y,w,h);
				cv::Mat im_roi = image(roi);
				roi.x = 0; roi.y = 0;
				cv::Mat mask_roi = mask(roi);
				cv::add(im_roi,cv::Scalar(80,0,0),im_roi,mask_roi);
			}

//			ROS_INFO("Print(x+3), (y+12)\n");
			int baseline = 0;
			int X = x+3, Y = y+12;
			cv::Size tsize = getTextSize(detect[i].label,cv::FONT_HERSHEY_SIMPLEX,1.0,2,&baseline);
			if( Y + tsize.height + 2 >= hmax) { Y = hmax - y - 12 - 2 - tsize.height;}
			if(Y < 0) Y = 0;
			if(X + tsize.width + 2 >= wmax) { X = wmax - x - 3 - 2 - tsize.width;}
			if(X < 0) X = 0;
			cv::putText(image,detect[i].label,cv::Point(X,Y),cv::FONT_HERSHEY_SIMPLEX,1.0,cv::Scalar(bl,gr,rd),2);
			std::string strscore = std::string("(") + tostr(detect[i].score) + std::string(")");
//			ROS_INFO("Print(x+6=%d, (y+24=%d)\n",x+6,y+24);
			X = x + 6; Y = y+24;
			tsize = getTextSize(strscore,cv::FONT_HERSHEY_SIMPLEX,0.4,1,&baseline);
			if( Y + tsize.height + 1 >= hmax) { Y = hmax - y - 24 - 1 - tsize.height;}
			if(Y < 0) Y = 0;
			if(X + tsize.width + 1 >= wmax) { X = wmax - x - 3 - 1 - tsize.width;}
			if(X < 0) X = 0;
			putText(image,strscore,cv::Point(X,Y),cv::FONT_HERSHEY_SIMPLEX,0.4,cv::Scalar(bl,gr,rd),1);
		}
		char buf[100];
//		sprintf(buf,"frame_%.3d.jpg",frame_idx++);
//		ROS_INFO("Writing(?) %s",buf);
//		cv::imwrite(buf, image);
//		ROS_INFO("imshow\n");
		cv::imshow("Image",image);
		cv::waitKey(40);


	}


private:
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	sensor_msgs::CvBridge bridge_;

	image_transport::SubscriberFilter image_sub_;
	message_filters::Subscriber<rein::DetectionArray> detections_sub_;
	message_filters::TimeSynchronizer<sensor_msgs::Image, rein::DetectionArray> sync_sub_;

};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "detection_displayer");

	DetectionDisplayer dd;
	ros::spin();
}
