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

#include "vfh_cluster_classifier/vfh_classifier_nodelet.h"
#include "vfh_cluster_classifier/vfh_classifier.h"

#include <pluginlib/class_list_macros.h>
#include <dynamic_reconfigure/server.h>


namespace vfh_classifier {

/**
 * Initializes the nodelet
 */
void VFHClassifierNodelet::childInit(ros::NodeHandle& nh)
{
	std::string dataset_location;
	if (!nh.getParam("dataset_location", dataset_location)) {
		ROS_ERROR("Parameter 'dataset_location' is missing");
	}
	boost::shared_ptr<VFHClassifier> detector = boost::make_shared<vfh_classifier::VFHClassifier>(dataset_location);

	detector_ = detector;
}



}

/**
 * Pluginlib declaration. This is needed for the nodelet to be dynamically loaded/unloaded
 */
typedef vfh_classifier::VFHClassifierNodelet VFHClassifierNodelet;
PLUGINLIB_DECLARE_CLASS (vfh_classifier, VFHClassifier, VFHClassifierNodelet, nodelet::Nodelet);
