#include "textured_object_detection/camera_info.hpp"

using namespace std;
using namespace cv;

void CameraInfo::init(string filename)
{
  sensor_msgs::CameraInfo ci;
  ifstream file;
  file.open(filename.c_str());
  file >> height >> width;
  ci.height = height;
  ci.width = width;
  for (int i = 0; i < 5; i++)
  {
    file >> D_values[i];
    ci.D[i] = D_values[i];
  }
  for (int i = 0; i < 9; i++)
  {
    file >> K_values[i];
    ci.K[i] = K_values[i];
  }
  for (int i = 0; i < 9; i++)
  {
    file >> R_values[i];
    ci.R[i] = R_values[i];
  }
  for (int i = 0; i < 12; i++)
  {
    file >> P_values[i];
    ci.P[i] = P_values[i];
  }
  file.close();
  cam_model.fromCameraInfo(ci);
}

void CameraInfo::rectify(const Mat& img, Mat& rectImg)
{
  rectImg.create(img.rows, img.cols, img.type());
  cv::Mat map1, map2;
  cv::initUndistortRectifyMap(K, D, R, P, cv::Size(width, height), CV_16SC2, map1, map2);
  cv::remap(img, rectImg, map1, map2, CV_INTER_LINEAR);
}

void CameraInfo::projectPoint(const Point3d& p3, Point2d& p2)
{
  cam_model.project3dToPixel(p3, p2);
}

void CameraInfo::projectPoint(const Point3f& p3, Point2f& p2)
{
  Point3d p3d(p3.x, p3.y, p3.z);
  Point2d p2d;
  cam_model.project3dToPixel(p3d, p2d);
  p2.x = (float)p2d.x;
  p2.y = (float)p2d.y;
}

void CameraInfo::getIntrinsicMatrix(Mat& intrinsic)
{
  cam_model.intrinsicMatrix().copyTo(intrinsic);
}

void CameraInfo::getDistortionMatrix(Mat& dist)
{
  cam_model.distortionCoeffs().copyTo(dist);
}

