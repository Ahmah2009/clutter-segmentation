#include <iostream>
#include "opencv/cv.h"
#include <stdlib.h>
#include "textured_object_detection/shared_functions.h"

using namespace cv;
using namespace std;

void saveCI(const Mat& d, const Mat& k, const Mat& r, const Mat& p, int height, int width, string filename)
{
  boost::shared_ptr<sensor_msgs::CameraInfo> ci(new sensor_msgs::CameraInfo());
  ci->height = height;
  ci->width = width;
  for (int i = 0; i < d.cols; i++)
    ci->D[i] = d.at<double> (0, i);

  for (int i = 0; i < k.rows; i++)
    for (int j = 0; j < k.cols; j++)
      ci->K[i * k.cols + j] = k.at<double> (i, j);

  for (int i = 0; i < r.rows; i++)
    for (int j = 0; j < r.cols; j++)
      ci->R[i * r.cols + j] = r.at<double> (i, j);

  for (int i = 0; i < p.rows; i++)
    for (int j = 0; j < p.cols; j++)
      ci->P[i * p.cols + j] = p.at<double> (i, j);
  saveCameraInfo(ci, filename);
}

int main(int argc, char** argv)
{
  if (argc != 5)
  {
    cout << "Format:" << endl;
    cout << argv[0] << " [height] [width] [intrinsics] [extrinsics]" << endl;
    return -1;
  }

  Mat d1, d2, m1, m2, r1, r2, p1, p2;
  FileStorage intrinsicsFs(argv[3], FileStorage::READ);
  if (intrinsicsFs.isOpened())
  {
    intrinsicsFs["D1"] >> d1;
    intrinsicsFs["D2"] >> d2;
    intrinsicsFs["M1"] >> m1;
    intrinsicsFs["M2"] >> m2;
    intrinsicsFs.release();
  }
  else
  {
    cout << "Can't read intrinsics file: " << argv[3] << endl;
    return -1;
  }

  FileStorage extrinsicsFs(argv[4], FileStorage::READ);
  if (extrinsicsFs.isOpened())
  {
    extrinsicsFs["R1"] >> r1;
    extrinsicsFs["R2"] >> r2;
    extrinsicsFs["P1"] >> p1;
    extrinsicsFs["P2"] >> p2;
    extrinsicsFs.release();
  }
  else
  {
    cout << "Can't read extrinsics file: " << argv[4] << endl;
    return -1;
  }
  int height = atoi(argv[1]), width = atoi(argv[2]);

  saveCI(d1, m1, r1, p1, height, width, "left_info.txt");
  saveCI(d2, m2, r2, p2, height, width, "right_info.txt");

  return 0;
}
