#include <textured_object_detection/gmap3D.hpp>

using namespace cv;

GMap3D::GMap3D(const Mat &_rvec, const Mat &_tvec, const Mat &_cameraMatirx, const Mat &_distCoeffs) :
  rvec(_rvec), tvec(_tvec), cameraMatrix(_cameraMatirx), distCoeffs(_distCoeffs)
{
}

void GMap3D::mapPoints(const std::vector<Point3f>& src, std::vector<Point2f>& dst) const
{
  vector < Point3f > rotated_object_points;
  project3dPoints(src, rvec, tvec, rotated_object_points);

  //TODO: get rid of code duplication with TOD

  double zavg = 0.0;
  for (size_t m = 0; m < rotated_object_points.size(); m++)
  {
    zavg += rotated_object_points[m].z;
  }

  if (zavg < 0)
  {
    for (size_t m = 0; m < rotated_object_points.size(); m++)
    {
      rotated_object_points[m] *= -1.0; //TODO: fix it
    }
  }

  dst.resize(rotated_object_points.size());
  Mat rvec1(3, 1, CV_64FC1, Scalar(0));
  Mat tvec1(3, 1, CV_64FC1, Scalar(0));
  projectPoints(Mat(rotated_object_points), rvec1, tvec1, cameraMatrix, distCoeffs, dst);
}
