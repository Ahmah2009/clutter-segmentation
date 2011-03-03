#include "textured_object_detection/shared_functions.h"
#include "opencv/cv.h"

using namespace std;
using namespace cv;

void createDirIsNeeded(string path)
{
  DIR* dir = opendir(path.c_str());
  if (dir == NULL)
  {
    mkdir(path.c_str(), 0777);
  }
  else
    closedir(dir);
}

void saveCameraInfo(const sensor_msgs::CameraInfoConstPtr& ci, string filename)
{
  ifstream file(filename.c_str());
  if (!file)
  {
    ofstream of(filename.c_str());
    of << ci->height << " " << ci->width << " ";
    for (int i = 0; i < 5; i++)
      of << fixed << setprecision(17) << ci->D[i] << " ";
    for (int i = 0; i < 9; i++)
      of << fixed << setprecision(17) << ci->K[i] << " ";
    for (int i = 0; i < 9; i++)
      of << fixed << setprecision(17) << ci->R[i] << " ";
    for (int i = 0; i < 11; i++)
      of << fixed << setprecision(17) << ci->P[i] << " ";
    of << fixed << setprecision(17) << ci->P[11];
    of.close();
  }
  else
  {
    file.close();
  }
}

void initQ(Mat& Q, const CameraInfo& right)
{
  Q.at<double> (0, 0) = Q.at<double> (1, 1) = 1.0;
  double Tx = -right.P.at<double> (0, 3) / right.P.at<double> (0, 0);
  Q.at<double> (3, 2) = 1.0 / Tx;
  Q.at<double> (0, 3) = -right.P.at<double> (0, 2);
  Q.at<double> (1, 3) = -right.P.at<double> (1, 2);
  Q.at<double> (2, 3) = right.P.at<double> (0, 0);
}

bool isValidPoint(const cv::Vec3f& pt)
{
  const double MISSING_Z = 10000.;
  // Check both for disparities explicitly marked as invalid (where OpenCV maps pt.z to MISSING_Z)
  // and zero disparities (point mapped to infinity).
  return pt[2] != MISSING_Z && !std::isinf(pt[2]);
}

void transformMaskUsingBiggestContour(Mat& mask, bool returnFilledContour)
{
  assert(mask.channels() == 1);
  vector<vector<Point> > contours;
  Mat temp;
  mask.copyTo(temp);
  findContours(temp, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

  int idxBiggestContour = -1;
  double maxArea = -1.0;
  for (size_t i = 0; i < contours.size(); i++)
  {
    double area = contourArea(Mat(contours[i]));
    if (area > maxArea)
    {
      maxArea = area;
      idxBiggestContour = i;
    }
  }

  const Point* points[1];
  points[0] = &contours[idxBiggestContour][0];
  int npts = contours[idxBiggestContour].size();
  Mat contour(mask.rows, mask.cols, CV_8UC1, Scalar(0)), result;
  fillPoly(contour, points, &npts, 1, Scalar(255));
  if (!returnFilledContour)
    bitwise_and(mask, contour, result);
  else
    contour.copyTo(result);
  result.copyTo(mask);
}
