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



#include <dpm_detector/dpm.h>

#include <octave/oct.h>
#include <octave/octave.h>
#include <octave/parse.h>

namespace {


void add_path(std::string path)
{
	octave_value_list in = octave_value(path);
	feval("addpath", in,1);
}


}


namespace dpm_detector {

DeformablePartModels::DeformablePartModels(std::string code_path, std::string models_path)
{
    string_vector argv (2);
    argv(0) = "embedded";
    argv(1) = "-q";
    octave_main (2, argv.c_str_vec(), 1);

    add_path(code_path);

    models_path_ = models_path;
    has_model_ = false;
}

void DeformablePartModels::detect()
{
	if (!has_model_) {
		ROS_WARN("No model loaded, not running detection.");
		return;
	}
	dim_vector dims;
	if (image_.channels()!=1) {
		dims = dim_vector(image_.rows, image_.cols, image_.channels());
	}
	else {
		dims = dim_vector(image_.rows, image_.cols);
	}
	uint8NDArray image_mat(dims);
	int rows = image_.rows;
	int cols = image_.cols;
	int channels = image_.channels();
	for (int i=0;i<rows;++i) {
		unsigned char* row = image_.ptr(i);
		for (int j=0;j<cols;++j) {
			for (int k=0;k<channels;++k) {
				image_mat(i,j,k) = row[j+channels*k];
			}
		}
	}

    octave_value_list in;
    in(0) = octave_value(image_mat);
    in(1) = octave_value(model_);
    in(2) = octave_value(recognition_threshold_);
    octave_value_list out;
    out = feval("detect2",in,1);

    Matrix bboxes = out(0).matrix_value();
    int n = bboxes.rows();
    detections_.detections.resize(n);
    for (octave_idx_type i=0;i<bboxes.rows();++i) {
    	detections_.detections[i].label = model_name_;
    	detections_.detections[i].detector = getName();
    	detections_.detections[i].mask.roi.x = bboxes(i,0);
    	detections_.detections[i].mask.roi.y = bboxes(i,1);
    	detections_.detections[i].mask.roi.width = bboxes(i,2) - bboxes(i,0);
    	detections_.detections[i].mask.roi.height = bboxes(i,3) - bboxes(i,1);
    	detections_.detections[i].score = bboxes(i,4);
    }
}

void DeformablePartModels::loadModels(const std::vector<std::string>& models)
{
	if (models.size()>0) {
		assert(models.size()==1); // DPM only loads one model at once
		setModelName(models[0]);

		std::string model_filename = models_path_+"/"+models[0]+".mat";
		octave_value_list in;
		in(0) = octave_value(model_filename);
		octave_value_list out;
		out = feval("load", in ,1);
		if (out.length()!=1) {
			ROS_ERROR("Cannot load model: %s",model_filename.c_str());
			has_model_ = false;
		}
		else {
			model_ = out(0).map_value();
			ROS_INFO("Loaded model %s", model_filename.c_str());
			has_model_ = true;
		}


	}

}

}
