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


#include "dpm_detector/dpm_nodelet.h"
#include "dpm_detector/dpm.h"

#include <pluginlib/class_list_macros.h>
#include <dynamic_reconfigure/server.h>

namespace dpm_detector {

/**
 * Initializes the nodelet
 */
void DPMNodelet::childInit(ros::NodeHandle& nh)
{
	std::string code_path;
	if (!nh.getParam("code_path", code_path)) {
		ROS_ERROR("Parameter 'code_path' is missing");
		ros::requestShutdown();
		exit(1);
	}
	std::string models_path;
	if (!nh.getParam("models_path", models_path)) {
		ROS_ERROR("Parameter 'models_path' is missing");
		ros::requestShutdown();
		exit(1);
	}

	// instantiate the detector
	detector_ = boost::make_shared<dpm_detector::DeformablePartModels>(code_path, models_path);
}

/**
 * Sets up dynamic reconfigure callback.
 * @param nh
 */
void DPMNodelet::initConfigureService(ros::NodeHandle& nh)
{
	static dynamic_reconfigure::Server<DPMConfig> config_srv(nh);
	dynamic_reconfigure::Server<DPMConfig>::CallbackType f = boost::bind(&DPMNodelet::configCallback, this, _1, _2);
	config_srv.setCallback(f);
}


/**
 * Callback for the configuration parameters. Automatically called when
 * a parameter is changed.
 * @param config
 * @param level
 */
void DPMNodelet::configCallback(DPMConfig &config, uint32_t level)
{
	float eps = 1e-5;

	boost::shared_ptr<dpm_detector::DeformablePartModels> detector =
			boost::static_pointer_cast<dpm_detector::DeformablePartModels>(detector_);


	double recognition_threshold = detector->getRecognitionThreshold();
	if (fabs(recognition_threshold - config.recognition_threshold)>eps) {
		NODELET_INFO ("[dpm_detector::%s::config_callback] Setting the recognition_threshold parameter to: %g.",
				detector->getName ().c_str (), config.recognition_threshold);
		detector->setRecognitionThreshold(config.recognition_threshold);
	}

	std::string model = detector->getModelName();
	if (model!=config.model) {
		NODELET_INFO ("[dpm_detector::%s::config_callback] Setting the model parameter to: %s.",
				detector->getName ().c_str (), config.model.c_str());
		std::vector<std::string> models(1);
		models[0] = config.model;
		detector->loadModels(models);
	}
}

}

/**
 * Pluginlib declaration. This is needed for the nodelet to be dynamically loaded/unloaded
 */
typedef dpm_detector::DPMNodelet DPMNodelet;
PLUGINLIB_DECLARE_CLASS (dpm_detector, DPMNodelet, DPMNodelet, nodelet::Nodelet);
