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

#include "rein/algorithms/tabletop_attention.h"



#include <cv_bridge/CvBridge.h>
#include <opencv2/highgui/highgui.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_ann.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/segmentation/extract_clusters.h>

namespace rein
{

struct PointUV
{
	uint32_t u;
	uint32_t v;
};

struct PointXYZRGBUV
{
	float x;
	float y;
	float z;
	float rgb;
	uint32_t u;
	uint32_t v;
};

}


POINT_CLOUD_REGISTER_POINT_STRUCT (rein::PointUV,
		(uint32_t, u, u)
		(uint32_t, v, v)
);

POINT_CLOUD_REGISTER_POINT_STRUCT (rein::PointXYZRGBUV,
		(float, x, x)
		(float, y, y)
		(float, z, z)
		(float, rgb, rgb)
		(uint32_t, u, u)
		(uint32_t, v, v)
);




namespace {

typedef rein::PointXYZRGBUV PointXYZRGBUV;
typedef rein::PointUV PointUV;

// declaring the following functions in the anonymous namespace makes
// them visible only within this compilation unit

void processCloud(const sensor_msgs::PointCloud2& cloud, std::vector<rein::PointUV>& indices)
{
//	ROS_INFO("Starting process on new cloud");
	typedef PointXYZRGBUV Point;

	// PCL objects
	pcl::KdTree<Point>::Ptr normals_tree_, clusters_tree_;

	pcl::VoxelGrid<Point> grid_, grid_objects_;
	pcl::PassThrough<Point> pass_;
	pcl::NormalEstimation<Point, pcl::Normal> n3d_;
	pcl::SACSegmentationFromNormals<Point, pcl::Normal> seg_;
	pcl::ProjectInliers<Point> proj_;
	pcl::ConvexHull2D<Point, Point> hull_;
	pcl::ExtractPolygonalPrismData<Point> prism_;
	pcl::EuclideanClusterExtraction<Point> pcl_cluster_;

	// Filtering parameters
	grid_.setLeafSize (0.01, 0.01, 0.01);             // 1cm voxel size for plane estimation
	grid_objects_.setLeafSize (0.003, 0.003, 0.003);  // 3mm voxel size for object clustering
	grid_.setFilterFieldName ("z");
	pass_.setFilterFieldName ("z");

	// restrict the Z dimension between 0.1m and 1.25m
	pass_.setFilterLimits (0.1, 1.25);
	grid_.setFilterLimits (0.4, 1.25);
	grid_.setDownsampleAllData (false);
	grid_objects_.setDownsampleAllData (false);

	normals_tree_ = boost::make_shared<pcl::KdTreeANN<Point> > ();
	clusters_tree_ = boost::make_shared<pcl::KdTreeANN<Point> > ();

	// Normal estimation parameters

	n3d_.setKSearch (10);                  // 10 k-neighbors by default
	n3d_.setSearchMethod (normals_tree_);

	// Table model fitting parameters
	seg_.setDistanceThreshold (0.05);             // 5cm
	seg_.setMaxIterations (10000);

	seg_.setNormalDistanceWeight (0.1);
	seg_.setOptimizeCoefficients (true);
	seg_.setModelType (pcl::SACMODEL_NORMAL_PLANE);
	seg_.setMethodType (pcl::SAC_RANSAC);
	seg_.setProbability (0.99);

	proj_.setModelType (pcl::SACMODEL_PLANE);

	// Consider objects starting at 2cm from the table and ending at 0.5m
	prism_.setHeightLimits (0.02, 0.50);

	// Clustering parameters
	// 3cm between two objects, 50 points per object cluster
	pcl_cluster_.setClusterTolerance (0.03);
	pcl_cluster_.setMinClusterSize (300);
	//pcl_cluster_.setMinClusterSize (100);
	pcl_cluster_.setSearchMethod (clusters_tree_);

	// Step 1 : Filter, remove NaNs and downsample
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_t = boost::make_shared< pcl::PointCloud<pcl::PointXYZRGB> >();
	pcl::fromROSMsg (cloud, *cloud_t);

	pcl::PointCloud<PointUV> cloud_coords; // cloud that will hold u,v image coordinates
	cloud_coords.points.resize(cloud_t->points.size());
	for (size_t i=0;i<cloud_coords.points.size();++i) {
		cloud_coords.points[i].u = i%cloud.width;
		cloud_coords.points[i].v = i/cloud.width;
	}

	pcl::PointCloud<Point>::Ptr cloud_merged = boost::make_shared< pcl::PointCloud<Point> >();

	pcl::concatenateFields(*cloud_t, cloud_coords, *cloud_merged);


	// filter NaNs and points out of range
	pcl::PointCloud<Point>::Ptr cloud_filtered =  boost::make_shared< pcl::PointCloud<Point> >();
	pass_.setInputCloud (cloud_merged);
	pass_.filter (*cloud_filtered);
//	ROS_INFO("Step 1 done");

	pcl::PointCloud<Point>::Ptr cloud_downsampled = boost::make_shared< pcl::PointCloud<Point> >();
	grid_.setInputCloud (cloud_filtered);
	grid_.filter (*cloud_downsampled);

	// Step 2 : Estimate normals
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals =  boost::make_shared<pcl::PointCloud<pcl::Normal> >();
	n3d_.setInputCloud (cloud_downsampled);
	n3d_.compute (*cloud_normals);
//	ROS_INFO("Step 2 done");

	// Step 3 : Perform planar segmentation
	pcl::PointIndices::Ptr table_inliers = boost::make_shared<pcl::PointIndices>();
	pcl::ModelCoefficients::Ptr table_coefficients = boost::make_shared<pcl::ModelCoefficients>();
	seg_.setInputCloud (cloud_downsampled);
	seg_.setInputNormals (cloud_normals);
	seg_.segment (*table_inliers, *table_coefficients);
	if (table_coefficients->values.size () > 3)
	{
//		ROS_INFO ("Model found with %d inliers: [%f %f %f %f].",
//				(int)table_inliers->indices.size (),
//				table_coefficients->values[0], table_coefficients->values[1],
//				table_coefficients->values[2], table_coefficients->values[3]);
	}
//	ROS_INFO("Step 3 done");

	if (table_inliers->indices.empty())
	{
		ROS_ERROR("Failed to detect table in scan");
		return;
	}

	// Step 4 : Project the table inliers on the table
	pcl::PointCloud<Point>::Ptr table_projected = boost::make_shared<pcl::PointCloud<Point> > ();
	proj_.setInputCloud (cloud_downsampled);
	proj_.setIndices (table_inliers);
	proj_.setModelCoefficients (table_coefficients);
	proj_.filter (*table_projected);
//	ROS_INFO("Step 4 done");

	// ---[ Estimate the convex hull
	pcl::PointCloud<Point>::Ptr table_hull =  boost::make_shared<pcl::PointCloud<Point> >();
	hull_.setInputCloud (table_projected);
	hull_.reconstruct (*table_hull);

	// ---[ Extract the remaining of all-table
	/*  pcl::PointCloud<Point> cloud_all_minus_table;
	  pcl::ExtractIndices<Point> extract_all_minus_table;
	  extract_all_minus_table.setInputCloud (cloud_filtered_ptr);
	  extract_all_minus_table.setIndices (table_inliers_ptr);
	  extract_all_minus_table.setNegative (true);
	  extract_all_minus_table.filter (cloud_all_minus_table);
	  boost::shared_ptr<const pcl::PointCloud<Point> > cloud_all_minus_table_ptr =
	    boost::make_shared<const pcl::PointCloud<Point> > (cloud_all_minus_table);
	 */


	// ---[ Get the objects on top of the table
	pcl::PointIndices::Ptr cloud_object_indices = boost::make_shared<pcl::PointIndices>();
	//prism_.setInputCloud (cloud_all_minus_table_ptr);
	prism_.setInputCloud (cloud_filtered);
	prism_.setInputPlanarHull (table_hull);
	prism_.segment (*cloud_object_indices);



	pcl::PointCloud<Point>::Ptr cloud_objects = boost::make_shared<pcl::PointCloud<Point> >();
	pcl::ExtractIndices<Point> extract_object_indices;
	//extract_object_indices.setInputCloud (cloud_all_minus_table_ptr);
	extract_object_indices.setInputCloud (cloud_filtered);
	extract_object_indices.setIndices (cloud_object_indices);
	extract_object_indices.filter (*cloud_objects);

	indices.resize(cloud_objects->points.size());
	for (size_t i=0;i<cloud_objects->points.size();++i) {
		indices[i].u =   cloud_objects->points[i].u;
		indices[i].v =   cloud_objects->points[i].v;
	}


	//if (cloud_objects.points.size () == 0)
	//  return;

	//	  // ---[ Downsample the points
	//	  pcl::PointCloud<Point>::Ptr cloud_objects_downsampled =  boost::make_shared <pcl::PointCloud<Point> >();
	//	  grid_objects_.setInputCloud (cloud_objects);
	//	  grid_objects_.filter (*cloud_objects_downsampled);
	//
	//
	//	  // ---[ Split the objects into Euclidean clusters
	//	  std::vector<pcl::PointIndices> clusters2;
	//	  //pcl_cluster_.setInputCloud (cloud_objects_ptr);
	//	  pcl_cluster_.setInputCloud (cloud_objects_downsampled);
	//	  pcl_cluster_.extract (clusters2);
	//	  ROS_INFO ("Number of clusters found matching the given constraints: %d.", (int)clusters2.size ());

}


void computeImageMask(const cv::Mat& image, const std::vector<PointUV>& indices, cv::Mat& mask)
{
	cv::Mat_<unsigned char>& mask_ = (cv::Mat_<unsigned char>&)mask;

	for (size_t i=0;i<indices.size();++i) {
		int x = indices[i].u;
		int y = indices[i].v;
		mask_(y,x) = 255;
	}
}



void displayMask(const cv::Mat& image, cv::Mat& mask)
{
	cv::Mat display_image(image.size(), CV_8UC3);
	cv::cvtColor(image,display_image, CV_GRAY2RGB);
	cv::Mat_<unsigned char>& mask_ = (cv::Mat_<unsigned char>&)mask;

	int nc = display_image.channels();
	for( int y = 0; y < display_image.rows; ++y ) {
		uchar* ptr = display_image.ptr<uchar>(y);
		for( int x = 0; x < display_image.cols; ++x ) {
			if (mask_(y,x)>0) {
				ptr[1] = cv::saturate_cast<uchar>(ptr[1]+50);
			}
			ptr += nc;
		}
	}

	cv::namedWindow("mask",1);
	cv::imshow("mask",display_image);
	cv::waitKey(40);
}

//#define DEBUG

void refineMasks(const cv::Mat& image, cv::Mat& mask, std::vector<cv::Rect>& rois, std::vector<cv::Mat>& masks)
{
	cv::dilate(mask,mask,cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(100,100)));
	cv::erode(mask,mask,cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(100,100)));

	cv::namedWindow("mask",1);
	cv::imshow("mask", mask);
	cv::waitKey(40);


	//find contours
	typedef std::vector<std::vector<cv::Point> > Contours;
	Contours contours;
	cv::Mat mask_contours;
	mask.copyTo(mask_contours); // create ned mask since it's going to be changes by findContours
	cv::findContours(mask_contours, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

	cv::Mat gc_image(image.size(), CV_8UC3);
	cv::cvtColor(image,gc_image, CV_GRAY2RGB);

//	printf("I have found %d countour\n", (int)contours.size());
	for (size_t i=0;i<contours.size();++i) {
		cv::Rect bbox = cv::boundingRect(cv::Mat(contours[i]));

		// is contour is too small, ignore it
		if (bbox.width<40 || bbox.height<40) continue;

		cv::rectangle(mask, cv::Point(bbox.x,bbox.y), cv::Point(bbox.x+bbox.width, bbox.y+bbox.height),cv::Scalar(255,0,0), 2);
		cv::namedWindow("mask",1);
		cv::imshow("mask", mask);
		cv::waitKey(40);


		cv::Mat crt_mask(mask.size(), mask.type());
		crt_mask = cv::Scalar(0);
		cv::Mat small_crt_mask = crt_mask(bbox);
		mask(bbox).copyTo(small_crt_mask);

//		cv::namedWindow("orig_mask",1);
//		cv::imshow("orig_mask", crt_mask);


		// extend the object bounding box
		// TODO: this is a hack, to be fixed
		bbox.height = std::min(bbox.height+20, image.rows-bbox.y);

		cv::Mat gc_mask(crt_mask.size(),CV_8UC1, cv::Scalar(cv::GC_BGD));
		gc_mask(bbox).setTo(cv::Scalar(cv::GC_PR_FGD));

		cv::erode(crt_mask,crt_mask,cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(30,30)));
		gc_mask.setTo(cv::Scalar(cv::GC_FGD), crt_mask);

		cv::Mat bgdModel;
		cv::Mat fgdModel;
		cv::grabCut(gc_image, gc_mask, bbox, bgdModel, fgdModel, 1, cv::GC_INIT_WITH_MASK);

		crt_mask = gc_mask & 1;

		Contours mask_contour;
		cv::findContours(crt_mask, mask_contour, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
		if (mask_contour.size()>0) {
			bbox = cv::boundingRect(cv::Mat(mask_contour[0]));
			rois.push_back(bbox);

			cv::Mat small_mask;
			crt_mask(bbox).copyTo(small_mask);
			masks.push_back(small_mask);
		}

#ifdef DEBUG
		cv::rectangle(gc_image, cv::Point(bbox.x,bbox.y), cv::Point(bbox.x+bbox.width, bbox.y+bbox.height),cv::Scalar(255,0,0), 2);
		cv::namedWindow("img",1);
		cv::imshow("img", gc_image);
		cv::waitKey(40);
#endif


#ifdef DEBUG
		displayMask(image,crt_mask);
#endif

	}

}

} // end of anonymous namespace





namespace rein {

void TabletopAttention::run()
{
	if (!point_cloud_->is_dense) {
		throw Exception("Expected a dense point cloud");
	}
	std::vector<PointUV> indices;
	processCloud(*point_cloud_, indices);

	cv::Mat mask(image_.size(), CV_8UC1, cv::Scalar::all(0));
	computeImageMask(image_, indices, mask);

	std::vector<cv::Rect> rois;
	std::vector<cv::Mat> masks;

	refineMasks(image_, mask, rois, masks);

	masks_.masks.clear();
	for (size_t i=0;i<rois.size();++i) {
		rein::Mask m;
		m.roi.x = rois[i].x;
		m.roi.y = rois[i].y;
		m.roi.width = rois[i].width;
		m.roi.height = rois[i].height;
		IplImage tmp_img = (IplImage)masks[i];
		m.mask = *sensor_msgs::CvBridge::cvToImgMsg(&tmp_img);
		masks_.masks.push_back(m);
	}
}

}
