#ifndef CAMERA_INFO_HPP_
#define CAMERA_INFO_HPP_
#include "opencv/cv.h"
#include <string>
#include <fstream>
#include <image_geometry/pinhole_camera_model.h>

struct CameraInfo
{
  int width, height;
  double D_values[5];
  double K_values[9];
  double R_values[9];
  double P_values[12];
  cv::Mat_<double> K, D, R, P;
  image_geometry::PinholeCameraModel cam_model;

  CameraInfo() :
    K(3, 3, &K_values[0]), D(1, 5, &D_values[0]), R(3, 3, &R_values[0]), P(3, 4, &P_values[0])
  {
  }

  void init(std::string filename);
  void rectify(const cv::Mat& img, cv::Mat& rectImg);
  void projectPoint(const cv::Point3d& p3, cv::Point2d& p2);
  void projectPoint(const cv::Point3f& p3, cv::Point2f& p2);

  void getIntrinsicMatrix(cv::Mat& intrinsic);
  void getDistortionMatrix(cv::Mat& dist);
};

#endif /* CAMERA_INFO_HPP_ */
