/**
 * Author: Julius Adorf
 */

#ifndef __TEST_H__

#include "clutseg/common.h"

#include <cv.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <opencv_candidate/PoseRT.h>

void sampleColorImage(cv::Mat & img);
void samplePose(opencv_candidate::PoseRT & pose);
void sampleCloud(PointCloudT & cloud);

#define __TEST_H__ __TEST_H__
#endif

