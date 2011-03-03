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

#include "rein/nodelets/detector.h"
#include "rein/core/detector.h"


#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <cv_bridge/CvBridge.h>

#include <rein/DetectionArray.h>
#include <geometry_msgs/PoseArray.h>
#include <rein/RectArray.h>
#include <time.h>
#include <boost/algorithm/string.hpp>

namespace rein {


/**
 * Subscribe to the topics of interest. Depending on
 * some parameters (e.g. use_rois) it can subscribe to different topics.
 */
void DetectorNodelet::subscribeTopics(ros::NodeHandle& nh)
{
	// get subscribe related parameters
	nh.getParam("use_image", use_image_);
	nh.getParam("use_masks", use_masks_);
	nh.getParam("use_input_detections", use_input_detections_);
	nh.getParam("use_point_cloud", use_point_cloud_);
	nh.getParam("queue_size", queue_size_);

	typedef message_filters::Subscriber<rein::MaskArray> MaskArraySubscriber;
	typedef message_filters::Subscriber<rein::DetectionArray> DetectionArraySubscriber;
	typedef message_filters::Subscriber<sensor_msgs::PointCloud2> PointCloud2Subscriber;

	// clearing the list unsubscribes from everything subscribed so far
	shared_ptrs_.clear();

	ROS_DEBUG("Detector nodelet about to subscribe to:\n"
			"\t image: %s\n"
			"\t masks: %s\n"
			"\t input_detections: %s\n"
			"\t point_cloud: %s\n", (use_image_?"Yes":"No"),(use_masks_?"Yes":"No"),
			(use_input_detections_?"Yes":"No"),(use_point_cloud_?"Yes":"No"));

	if (use_image_) {
		boost::shared_ptr<image_transport::ImageTransport> it = make_shared<image_transport::ImageTransport>(nh);
		boost::shared_ptr<image_transport::SubscriberFilter> image_sub =  make_shared<image_transport::SubscriberFilter>();
		image_sub->subscribe(*it, "image", queue_size_);

		if (use_point_cloud_) {
			boost::shared_ptr<PointCloud2Subscriber> point_cloud_sub =  make_shared<PointCloud2Subscriber>();
			point_cloud_sub->subscribe(nh, "point_cloud", queue_size_);

			if (use_input_detections_) {
				boost::shared_ptr<DetectionArraySubscriber> detections_sub =  make_shared<DetectionArraySubscriber>();
				detections_sub->subscribe(nh, "input_detections", queue_size_);

				typedef message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::PointCloud2, rein::DetectionArray> SynchronizerImagePointCloudDetections;
				boost::shared_ptr<SynchronizerImagePointCloudDetections> sync_sub = make_shared<SynchronizerImagePointCloudDetections>(queue_size_+2);
				sync_sub->connectInput(*image_sub, *point_cloud_sub, *detections_sub);
				sync_sub->registerCallback(boost::bind(&DetectorNodelet::dataCallback, this, _1, _2, MaskArrayConstPtr(), _3));

			}
			else {
				if (use_masks_) {
					boost::shared_ptr<MaskArraySubscriber> masks_sub =  make_shared<MaskArraySubscriber>();
					masks_sub->subscribe(nh, "masks", queue_size_);

					typedef message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::PointCloud2, rein::MaskArray> SynchronizerImagePointCloudMasks;
					boost::shared_ptr<SynchronizerImagePointCloudMasks> sync_sub = make_shared<SynchronizerImagePointCloudMasks>(queue_size_+2);
					sync_sub->connectInput(*image_sub, *point_cloud_sub, *masks_sub);
					sync_sub->registerCallback(boost::bind(&DetectorNodelet::dataCallback, this, _1, _2, _3, DetectionArrayConstPtr()));
				}
				else {
					typedef message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::PointCloud2> SynchronizerImagePointCloud;
					boost::shared_ptr<SynchronizerImagePointCloud> sync_sub = make_shared<SynchronizerImagePointCloud>(queue_size_+2);
					sync_sub->connectInput(*image_sub, *point_cloud_sub);
					sync_sub->registerCallback(boost::bind(&DetectorNodelet::dataCallback, this, _1, _2, MaskArrayConstPtr(), DetectionArrayConstPtr()));
				}
			}
		}
		else {
			if (use_input_detections_) {
				boost::shared_ptr<DetectionArraySubscriber> detections_sub =  make_shared<DetectionArraySubscriber>();
				detections_sub->subscribe(nh, "input_detections", queue_size_);

				typedef message_filters::TimeSynchronizer<sensor_msgs::Image, rein::DetectionArray> SynchronizerImageDetections;
				boost::shared_ptr<SynchronizerImageDetections> sync_sub = make_shared<SynchronizerImageDetections>(queue_size_+2);
				sync_sub->connectInput(*image_sub, *detections_sub);
				sync_sub->registerCallback(boost::bind(&DetectorNodelet::dataCallback, this, _1, sensor_msgs::PointCloud2ConstPtr(), MaskArrayConstPtr(), _2));
			}
			else {

				if (use_masks_) {
					boost::shared_ptr<MaskArraySubscriber> masks_sub =  make_shared<MaskArraySubscriber>();
					masks_sub->subscribe(nh, "masks", queue_size_);

					typedef message_filters::TimeSynchronizer<sensor_msgs::Image, rein::MaskArray> SynchronizerImageMaskArray;
					boost::shared_ptr<SynchronizerImageMaskArray> sync_sub = make_shared<SynchronizerImageMaskArray>(queue_size_+2);
					sync_sub->connectInput(*image_sub, *masks_sub);
					sync_sub->registerCallback(boost::bind(&DetectorNodelet::dataCallback, this, _1, sensor_msgs::PointCloud2ConstPtr(), _2, DetectionArrayConstPtr()));
				}
				else {
					image_sub->registerCallback(boost::bind(&DetectorNodelet::dataCallback, this,_1, sensor_msgs::PointCloud2ConstPtr(), MaskArrayConstPtr(), DetectionArrayConstPtr()));
				}
			}
		}
	}
	else {
		if (use_point_cloud_) {
			boost::shared_ptr<PointCloud2Subscriber> point_cloud_sub =  make_shared<PointCloud2Subscriber>();
			point_cloud_sub->subscribe(nh, "point_cloud", queue_size_);

			if (use_input_detections_) {
				boost::shared_ptr<DetectionArraySubscriber> detections_sub =  make_shared<DetectionArraySubscriber>();
				detections_sub->subscribe(nh, "input_detections", queue_size_);

				typedef message_filters::TimeSynchronizer<sensor_msgs::PointCloud2, rein::DetectionArray> SynchronizerPointCloudDetections;
				boost::shared_ptr<SynchronizerPointCloudDetections> sync_sub = make_shared<SynchronizerPointCloudDetections>(queue_size_+2);
				sync_sub->connectInput(*point_cloud_sub, *detections_sub);
				sync_sub->registerCallback(boost::bind(&DetectorNodelet::dataCallback, this, sensor_msgs::ImageConstPtr(), _1, MaskArrayConstPtr(), _2));
			}
			else {
				if (use_masks_) {
					boost::shared_ptr<MaskArraySubscriber> masks_sub =  make_shared<MaskArraySubscriber>();
					masks_sub->subscribe(nh, "masks", queue_size_);

					typedef message_filters::TimeSynchronizer<sensor_msgs::PointCloud2, rein::MaskArray> SynchronizerPointCloudROI;
					boost::shared_ptr<SynchronizerPointCloudROI> sync_sub = make_shared<SynchronizerPointCloudROI>(queue_size_+2);
					sync_sub->connectInput(*point_cloud_sub, *masks_sub);
					sync_sub->registerCallback(boost::bind(&DetectorNodelet::dataCallback, this, sensor_msgs::ImageConstPtr(), _1, _2, DetectionArrayConstPtr()));
				}
				else {
					point_cloud_sub->registerCallback(boost::bind(&DetectorNodelet::dataCallback, this, sensor_msgs::ImageConstPtr(), _1, MaskArrayConstPtr(), DetectionArrayConstPtr()));
				}
			}
		}
	}
}


void DetectorNodelet::advertiseTopics(ros::NodeHandle& nh)
{
	// advertise published topics
	detections_pub_ = nh.advertise<DetectionArray>("detections",1);
	rois_pub_ = nh.advertise<MaskArray>("rois",1);
	masks_pub_ = nh.advertise<MaskArray>("masks",1);
	poses_pub_ = nh.advertise<MaskArray>("poses",1);
}

/**
 * Checks to see if anybody is interested in the results.
 * @return True is somebody subscribed to results
 */
bool DetectorNodelet::resultsNeeded()
{
	return (detections_pub_.getNumSubscribers()>0 || rois_pub_.getNumSubscribers()>0 || masks_pub_.getNumSubscribers()>0 || poses_pub_.getNumSubscribers()>0 );
}

/**
 * Callback
 * @param image the image to run recognition on
 * @param rois the regions of interest in the image (might be missing)
 */
void DetectorNodelet::dataCallback(const sensor_msgs::ImageConstPtr& image,
									const sensor_msgs::PointCloud2ConstPtr& point_cloud,
									const rein::MaskArrayConstPtr& masks,
									const rein::DetectionArrayConstPtr& detections)
{
	static sensor_msgs::CvBridge cv_bridge;
	// convert input image to OpenCV Mat

	if (use_image_) {
		header_.header = image->header;
	}
	else if (use_point_cloud_){
		header_.header = point_cloud->header;
	}

    if (use_image_) detector_->setImage(cv_bridge.imgMsgToCv(image));
	if (use_masks_) detector_->setMasks(masks);
	if (use_input_detections_) detector_->setInputdetections(detections);
	if (use_point_cloud_) detector_->setPointCloud(point_cloud);

	if (resultsNeeded()) {
		runNodelet();
		publishResults();
	}
}


/**
 * Runs the detector.
 */
void DetectorNodelet::runNodelet()
{
	NODELET_DEBUG("Running %s detector", detector_->getName().c_str());
	clock_t start = clock();
	detector_->detect();
	NODELET_DEBUG("Detection took %g seconds.", ((double)clock()-start)/CLOCKS_PER_SEC);
}


/**
 * Publishes detection results
 */
void DetectorNodelet::publishResults()
{
	rein::DetectionArrayPtr detections = boost::make_shared<DetectionArray>(detector_->getDetections());
	// publish detections
	detections_pub_.publish(detections);

	if (masks_pub_.getNumSubscribers()>0) {

		MaskArrayPtr masks = boost::make_shared<MaskArray>();
		masks->masks.resize(detections->detections.size());

		for (size_t i=0;i<detections->detections.size();++i) {
			masks->masks[i] = detections->detections[i].mask;
		}
		masks->header = header_.header;
		masks_pub_.publish(masks);

	}

	if (rois_pub_.getNumSubscribers()>0) {
		// publish bounding boxes

		RectArrayPtr rois = boost::make_shared<RectArray>();
		rois->rects.resize(detections->detections.size());

		for (size_t i=0;i<detections->detections.size();++i) {
			rois->rects[i] = detections->detections[i].mask.roi;
		}
		rois->header = header_.header;
		masks_pub_.publish(rois);
	}

	if (poses_pub_.getNumSubscribers()>0) {
		geometry_msgs::PoseArrayPtr poses = boost::make_shared<geometry_msgs::PoseArray>();
		poses->poses.resize(detections->detections.size());

		for (size_t i=0;i<detections->detections.size();++i) {
			poses->poses[i] = detections->detections[i].pose.pose;

			if (i==0) {
				poses->header = detections->detections[i].pose.header;
			}
		}
		poses_pub_.publish(poses);
	}
}


/**
 * Loads a list of models into the detector
 * @param models  comma-separated list of models
 */
void DetectorNodelet::loadModels(const std::string& models)
{
	if (models=="__none__") return;
	std::vector<std::string> models_vec;
	boost::split(models_vec, models, boost::is_any_of("\t ,"));
	std::vector<std::string> models_vec_filtered;
	for (size_t i=0;i<models_vec.size();++i) {
		if (!models_vec[i].empty()) models_vec_filtered.push_back(models_vec[i]);
	}

	if (models_vec_filtered.size()!=0) {
		detector_->loadModels(models_vec_filtered);
	}
	else {
		detector_->loadAllModels();
	}
}

}
