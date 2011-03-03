#include <iostream>
#include "posest/pnp_ransac.h"
#include "stereo_object_recognition/RANSACRigidTransformationEstimator.h"
#include "stereo_object_recognition/SVDRigidEstimator.h"
using namespace std;
using namespace cv;
using namespace stereo_object_recognition;

void generate3DPointCloud(vector<Point3f>& points, Point3f pmin = Point3f(0.5, 0.5, 0), Point3f pmax = Point3f(1, 1, 0.5))
{
  const Point3f delta = pmax - pmin;
  for (size_t i = 0; i < points.size(); i++)
  {
    Point3f p(float(rand()) / RAND_MAX, float(rand()) / RAND_MAX, float(rand()) / RAND_MAX);
    p.x *= delta.x;
    p.y *= delta.y;
    p.z *= delta.z;
    p = p + pmin;
    points[i] = p;
  }
}

int main(int argc, char** argv)
{
  double t = (double)getTickCount();
  for (int kk = 0; kk < 10; kk++)
  {
    Mat rvec1 = Mat::zeros(3, 1, CV_64FC1);
    Mat tvec1 = Mat::zeros(3, 1, CV_64FC1);
    rvec1.at<double> (0, 0) = 1.0f;
    tvec1.at<double> (0, 0) = 1.0f;
    tvec1.at<double> (1, 0) = 2.0f;

    vector<Point3f> points;
    points.resize(500);
    generate3DPointCloud(points);

    vector<Point3f> rtpoints;
    project3dPoints(points, rvec1, tvec1, rtpoints);

    for (size_t i = 0; i < rtpoints.size(); i++)
    {
      if (i % 20 == 0)
      {
        rtpoints[i] = rtpoints[rand() % rtpoints.size()];
      }
    }


    Mat rvec(3, 1, CV_64F);
    Mat tvec(3, 1, CV_64F);
    vector<int> inliers;

    SVDRigidEstimator estimator;
    RANSACRigidTransformationEstimator ransacEstimator( estimator, 0.01, 0.999, 0.9, 10);
    std::vector<std::pair<cv::Point3d,cv::Point3d> > pairs;
    for (size_t pairIndex = 0; pairIndex < points.size(); pairIndex++)
    {
      pairs.push_back(make_pair(Point3d(points[pairIndex].x, points[pairIndex].y, points[pairIndex].z),
           Point3d(rtpoints[pairIndex].x, rtpoints[pairIndex].y, rtpoints[pairIndex].z)));
    }
    Mat R;
    Vec3d t;
    bool isEstimateSuccess = ransacEstimator.estimate( pairs, R, t);
    cout << "RANSAC estimation status = " << isEstimateSuccess << endl;
    if (isEstimateSuccess)
    {
      rvec.create(3, 1, CV_64FC1);
      tvec.create(3, 1, CV_64FC1);
      tvec.at<double> (0, 0) = t[0];
      tvec.at<double> (1, 0) = t[1];
      tvec.at<double> (2, 0) = t[2];
      Rodrigues(R, rvec);
      vector<unsigned> estInliers = ransacEstimator.getInliers();
      for (size_t inliersIndex = 0; inliersIndex < estInliers.size(); inliersIndex++)
      {
          inliers.push_back(estInliers[inliersIndex]);
      }
    }

    bool isTestSuccess = inliers.size() == 475;
    isTestSuccess = isTestSuccess && isEstimateSuccess;
    double eps = 1.0e-7;
    isTestSuccess = isTestSuccess && (abs(rvec.at<double> (0, 0) - 1) < eps);
    isTestSuccess = isTestSuccess && (abs(rvec.at<double> (1, 0)) < eps);
    isTestSuccess = isTestSuccess && (abs(rvec.at<double> (2, 0)) < eps);
    isTestSuccess = isTestSuccess && (abs(tvec.at<double> (0, 0) - 1) < eps);
    isTestSuccess = isTestSuccess && (abs(tvec.at<double> (1, 0) - 2) < eps);
    isTestSuccess = isTestSuccess && (abs(tvec.at<double> (2, 0)) < eps);
    if (!isTestSuccess)
    {
      cout << "rvec" << "\t" << "tvec" << endl;
      for (int i = 0; i < 3; i++)
      {
        cout << rvec.at<double> (i, 0) << "\t" << tvec.at<double> (i, 0) << endl;
      }
      cout << "Inliers count = " << inliers.size() << endl;
      cout << "Test failed" << endl;
    }
    else
      cout << "Test successful" << endl;
  }
  t = ((double)getTickCount() - t) / getTickFrequency();
  cout << "All time = " << t << endl;
  return 0;
}
