/*
 * error_sample.cpp
 *
 *  Created on: 23.09.2010
 *      Author: alex
 */

#include "cv.h"
#include <fstream>
#include <iostream>
using namespace std;
using namespace cv;

void project3dPoints(const vector<Point3f>& points, const Mat& rvec, const Mat& tvec, vector<Point3f>& modif_points)
{
  modif_points.clear();
  modif_points.resize(points.size());
  Mat R(3, 3, CV_64FC1);
  Rodrigues(rvec, R);
  for (size_t i = 0; i < points.size(); i++)
  {
    modif_points[i].x = R.at<double> (0, 0) * points[i].x + R.at<double> (0, 1) * points[i].y + R.at<double> (0, 2)
        * points[i].z + tvec.at<double> (0, 0);
    modif_points[i].y = R.at<double> (1, 0) * points[i].x + R.at<double> (1, 1) * points[i].y + R.at<double> (1, 2)
        * points[i].z + tvec.at<double> (1, 0);
    modif_points[i].z = R.at<double> (2, 0) * points[i].x + R.at<double> (2, 1) * points[i].y + R.at<double> (2, 2)
        * points[i].z + tvec.at<double> (2, 0);
  }
}

int main(int argc, char* argv[])
{
  vector<Point3f> object_points;

  string indexStr = "";
  if (argc > 1)
    indexStr = argv[1];

  ifstream obj_file;
  obj_file.open(("object" + indexStr + ".txt").c_str());
  if (obj_file.is_open() == false)
  {
    cout << "File object.txt not found, exiting!" << endl;
    exit(-1);
  }
  while (!obj_file.eof())
  {
    Point3f p;
    obj_file >> p.x >> p.y >> p.z;
    object_points.push_back(p);
  }
  obj_file.close();

  vector<Point2f> image_points;

  ifstream img_file;
  img_file.open(("image" + indexStr + ".txt").c_str());
  if (img_file.is_open() == false)
  {
    cout << "File image.txt not found, exiting!" << endl;
    exit(-1);
  }
  while (!img_file.eof())
  {
    Point2f p;
    img_file >> p.x >> p.y;
    image_points.push_back(p);
  }
  img_file.close();

  assert(image_points.size() == object_points.size());

  cout << image_points.size() << "\t" << object_points.size() << endl;

  ifstream cam_file;
  cam_file.open(("cam" + indexStr + ".txt").c_str());
  if (cam_file.is_open() == false)
  {
    cout << "File cam.txt not found, exiting!" << endl;
    exit(-1);
  }
  Mat camera_matrix(3, 3, CV_32FC1);
  cam_file >> camera_matrix.at<float> (0, 0) >> camera_matrix.at<float> (0, 1) >> camera_matrix.at<float> (0, 2);
  cam_file >> camera_matrix.at<float> (1, 0) >> camera_matrix.at<float> (1, 1) >> camera_matrix.at<float> (1, 2);
  cam_file >> camera_matrix.at<float> (2, 0) >> camera_matrix.at<float> (2, 1) >> camera_matrix.at<float> (2, 2);
  Mat dist_coeffs(1, 5, CV_32FC1);
  for (int j = 0; j < 5; j++)
    cam_file >> dist_coeffs.at<float> (0, j);
  cam_file.close();

  vector<int> inliers;
  Mat rvec, tvec;
  //run solvePnPRansac for finding inliers (using point clouds from train base)
  solvePnP(Mat(object_points), Mat(image_points), camera_matrix, dist_coeffs, rvec, tvec);

  vector<Point3f> modif_points;
  project3dPoints(object_points, rvec, tvec, modif_points);
  for (size_t index = 0; index < modif_points.size(); index++)
    cout << modif_points[index].x << " " << modif_points[index].y << " " << modif_points[index].z << endl;
}
