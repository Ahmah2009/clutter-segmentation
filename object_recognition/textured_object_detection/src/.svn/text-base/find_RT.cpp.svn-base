#include "opencv/cv.h"
#include "opencv/highgui.h"
#include <iostream>
#include "textured_object_detection/shared_functions.h"
#include <ros/ros.h>
#include "posest/pnp_ransac.h"

using namespace cv;
using namespace std;

int main(int argc, char** argv)
{
  if (argc != 3)
  {
    cout << "Format:" << endl;
    cout << argv[0] << " [directory(should contain left, right,crop subfolders)] [count]" << endl;
    return -1;
  }

  const string resultFile = "transform.xml";
  int count = atoi(argv[2]);

  string lciPath, rciPath, prociPath;
  CameraInfo leftInfo, rightInfo, proInfo;
  lciPath = string(argv[1]) + "/left_info.txt";
  leftInfo.init(lciPath);
  rciPath = string(argv[1]) + "/right_info.txt";
  rightInfo.init(rciPath);
  Mat_<double> Q(4, 4, 0.0);
  initQ(Q, rightInfo);
  prociPath = string(argv[1]) + "/pro_info.txt";
  proInfo.init(prociPath);

  Mat leftIm, proIm;
  Mat leftImTex, rightImTex;
  Mat leftRTex, rightRTex;

  char filename[256];
  string config_str;

  int imageIndex = 1;

  Mat resultRvec, resultTvec;

  vector<Point3f> objectPoints;
  vector<Point2f> imagePoints;

  while (imageIndex <= count)
  {
    config_str = string(argv[1]) + "/left_tex/%d.png";
    sprintf(filename, config_str.c_str(), imageIndex);
    leftImTex = imread(filename, 0);
    if (leftImTex.data == NULL)
      break;
    cout << filename << endl;

    config_str = string(argv[1]) + "/right_tex/%d.png";
    sprintf(filename, config_str.c_str(), imageIndex);

    rightImTex = imread(filename, 0);
    if (rightImTex.data == NULL)
      break;
    cout << filename << endl;

    config_str = string(argv[1]) + "/left/%d.png";
    sprintf(filename, config_str.c_str(), imageIndex);

    leftIm = imread(filename, 0);
    if (leftIm.data == NULL)
      break;
    cout << filename << endl;

    config_str = string(argv[1]) + "/pro/%d.png";
    sprintf(filename, config_str.c_str(), imageIndex);

    proIm = imread(filename, 0);
    if (proIm.data == NULL)
      break;
    cout << filename << endl;

    leftImTex.copyTo(leftRTex);
    leftInfo.rectify(leftImTex, leftRTex);
    Mat leftR;
    leftInfo.rectify(leftIm, leftR);

    rightImTex.copyTo(rightRTex);
    rightInfo.rectify(rightImTex, rightRTex);

    Mat d, tmp;
    StereoBM(StereoBM::BASIC_PRESET, 128, 15)(leftRTex, rightRTex, d);
    d.convertTo(tmp, CV_8U, 1.0 / 16.0);

    Mat_<Vec3f> point_cloud;
    reprojectImageTo3D(d, point_cloud, Q, true);

    Size boardSize(6, 8);
    vector<Point2f> leftPointbuf;
    bool leftFound = findChessboardCorners(leftR, boardSize, leftPointbuf, CV_CALIB_CB_ADAPTIVE_THRESH );
    vector<Point2f> proPointbuf;
    bool proFound = findChessboardCorners(proIm, boardSize, proPointbuf, CV_CALIB_CB_ADAPTIVE_THRESH );

    if (!leftFound || !proFound)
    {
      imageIndex++;
      continue;
    }

    for (size_t index = 0; index < leftPointbuf.size(); index++)
    {
      if (isValidPoint(point_cloud.at<Vec3f> (leftPointbuf[index].y, leftPointbuf[index].x)))
      {
        objectPoints.push_back(Point3f(point_cloud.at<Vec3f> (leftPointbuf[index].y, leftPointbuf[index].x)[0],
                                       point_cloud.at<Vec3f> (leftPointbuf[index].y, leftPointbuf[index].x)[1],
                                       point_cloud.at<Vec3f> (leftPointbuf[index].y, leftPointbuf[index].x)[2]));
        imagePoints.push_back(proPointbuf[index]);
      }
    }
    imageIndex++;
    cout << "Corners count = " << objectPoints.size() << endl;
  }

  if (objectPoints.size() < 4)
  {
    cout << "Too small corners count! You should use some more images." << endl;
  }

  Mat rvec, tvec;
  solvePnP(Mat(objectPoints), Mat(imagePoints), proInfo.K, proInfo.D, rvec, tvec);

  cout << endl << "Resulting transformation: " << endl;
  cout << "rvec " << rvec.at<double>(0, 0) << " " << rvec.at<double>(0, 1) << " " << rvec.at<double>(0, 2) << endl;
  cout << "tvec " << tvec.at<double>(0, 0) << " " << tvec.at<double>(0, 1) << " " << tvec.at<double>(0, 2) << endl << endl;

  vector<Point3f> rotated_object_points;
  project3dPoints(objectPoints, rvec, tvec, rotated_object_points);

  vector<Point2f> projected_points;
  projected_points.resize(rotated_object_points.size());
  Mat rvec1, tvec1;
  rvec1.create(3, 1, CV_64FC1);
  tvec1.create(3, 1, CV_64FC1);
  rvec1.at<double> (0, 0) = rvec1.at<double> (1, 0) = rvec1.at<double> (2, 0) = 0.0;
  tvec1.at<double> (0, 0) = tvec1.at<double> (1, 0) = tvec1.at<double> (2, 0) = 0.0;
  projectPoints(Mat(rotated_object_points), rvec1, tvec1, proInfo.K, proInfo.D, projected_points);

  float error = 0;
  float avg_error = 0;
  cout << "Projected points count " << projected_points.size() << endl;
  for (size_t index = 0; index < projected_points.size(); index++)
  {
    float dist = norm(projected_points[index] - imagePoints[index]);
    if(dist > error)
    {
      error = dist;
    }

    avg_error += dist*dist;
  }

  cout << "Max error = " << error << endl;
  cout << "Average error = " << sqrt(avg_error/projected_points.size()) << endl;

  FileStorage fs(resultFile.c_str(), FileStorage::WRITE);
  fs << "rvec" << rvec;
  fs << "tvec" << tvec;
  fs.release();

  return 0;
}
