/**
 * Author: Julius Adorf
 */

#ifndef __TEST_H__

#include "clutseg/common.h"

#include <cv.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <opencv_candidate/PoseRT.h>

#define SKIP_IF_FAST if (fast()) { fast_warning(); return; }

bool fast();
void fast_warning();
void imshow_and_wait(const std::string & name, const cv::Mat & canvas);
void sampleColorImage(cv::Mat & img);
void samplePose(opencv_candidate::PoseRT & pose);
void sampleCloud(clutseg::PointCloudT & cloud);

#define __TEST_H__ __TEST_H__
#endif

