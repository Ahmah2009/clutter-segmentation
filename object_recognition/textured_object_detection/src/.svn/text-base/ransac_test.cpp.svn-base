#include <iostream>
#include "posest/pnp_ransac.h"

using namespace std;
using namespace cv;

void generate3DPointCloud(vector<Point3f>& points, Point3f pmin = Point3f(-1,
		-1, 5), Point3f pmax = Point3f(1, 1, 10))
{
	const Point3f delta = pmax - pmin;
	for (size_t i = 0; i < points.size(); i++)
	{
		Point3f p(float(rand()) / RAND_MAX, float(rand()) / RAND_MAX,
				float(rand()) / RAND_MAX);
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
	for (int kk = 0; kk<  10; kk++)
	{
		Mat intrinsics = Mat::eye(3, 3, CV_32FC1);
		intrinsics.at<float> (0, 0) = 400.0;
		intrinsics.at<float> (1, 1) = 400.0;
		intrinsics.at<float> (0, 2) = 640 / 2;
		intrinsics.at<float> (1, 2) = 480 / 2;
		Mat dist_coeffs = Mat::zeros(5, 1, CV_32FC1);

		Mat rvec1 = Mat::zeros(3, 1, CV_64FC1);
		Mat tvec1 = Mat::zeros(3, 1, CV_64FC1);
		rvec1.at<double> (0, 0) = 1.0f;

		tvec1.at<double> (0, 0) = 1.0f;
		tvec1.at<double> (1, 0) = 1.0f;

		vector<Point3f> points;
		points.resize(500);
		generate3DPointCloud(points);

		vector<Point2f> points1;
		points1.resize(points.size());

		projectPoints(Mat(points), rvec1, tvec1, intrinsics, dist_coeffs,
				points1);

		for (size_t i = 0; i < points1.size(); i++) {
			if (i % 20 == 0) {
				points1[i] = points1[rand() % points.size()];
			}
		}

		Mat rvec(3, 1, CV_64F);
		Mat tvec(3, 1, CV_64F);
		vector<int> inliers;

		solvePnPRansac(points, points1, intrinsics, dist_coeffs, rvec, tvec,
				false, 1000, 2.0, -1, &inliers);

		bool isTestSuccess = inliers.size() == 475;
		double eps = 1.0e-7;
		isTestSuccess = isTestSuccess
				&& (abs(rvec.at<double> (0, 0) - 1) < eps);
		isTestSuccess = isTestSuccess && (abs(rvec.at<double> (1, 0)) < eps);
		isTestSuccess = isTestSuccess && (abs(rvec.at<double> (2, 0)) < eps);
		isTestSuccess = isTestSuccess
				&& (abs(tvec.at<double> (0, 0) - 1) < eps);
		isTestSuccess = isTestSuccess
				&& (abs(tvec.at<double> (1, 0) - 1) < eps);
		isTestSuccess = isTestSuccess && (abs(tvec.at<double> (2, 0)) < eps);
		if (!isTestSuccess)
		{
			cout << "rvec" << "\t" << "tvec" << endl;
					for (int i = 0; i < 3; i++) {
						cout << rvec.at<double> (i, 0) << "\t" << tvec.at<double> (i, 0)
								<< endl;
					}
					cout << "Inliers count = " << inliers.size() << endl;
			cout << "Test failed" << endl;
		}
		else
			cout << "Test successful" << endl;
	}
	t = ((double)getTickCount() - t)/getTickFrequency();
	cout << "All time = " << t << endl;
	return 0;
}
