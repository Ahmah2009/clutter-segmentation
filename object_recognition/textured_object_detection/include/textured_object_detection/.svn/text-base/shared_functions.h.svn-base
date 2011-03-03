#ifndef SHARED_FUNCTIONS_H_
#define SHARED_FUNCTIONS_H_

#include <sys/stat.h>
#include <dirent.h>
#include <string>
#include <fstream>
#include <iomanip>
#include <sensor_msgs/CameraInfo.h>
#include "textured_object_detection/camera_info.hpp"

void createDirIsNeeded(std::string path);
void saveCameraInfo(const sensor_msgs::CameraInfoConstPtr& ci, std::string filename);
void initQ(cv::Mat& Q, const CameraInfo& right);
bool isValidPoint(const cv::Vec3f& pt);
void transformMaskUsingBiggestContour(cv::Mat& mask, bool returnFilledContour);

#endif /* SHARED_FUNCTIONS_H_ */
