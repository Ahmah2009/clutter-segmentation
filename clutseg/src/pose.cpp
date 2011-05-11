/*
 * Author: Julius Adorf
 */

#include "clutseg/pose.h"

#include <fiducial/fiducial.h>
#include <opencv/cxeigen.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random.hpp>
#include <boost/foreach.hpp>
#include <boost/format.hpp>

using namespace fiducial;
using namespace cv;
using namespace opencv_candidate;
using namespace boost;

namespace clutseg {

    Point projectOrigin(const PoseRT & pose, const opencv_candidate::Camera & camera) {
        Point3d o(0, 0, 0);
        vector<Point3f> op(1);
        vector<Point2f> ip(1);
        op[0] = o;
        projectPoints(Mat(op), pose.rvec, pose.tvec, camera.K, camera.D, ip);
        return ip[0];
    }
    
    void randomizePose(PoseRT & pose, double stddev_t, double stddev_r) {
        mt19937 twister; 
        normal_distribution<> n_t(0, stddev_t);
        variate_generator<mt19937&, normal_distribution<> > noise_t(twister, n_t);
        pose.tvec.at<double>(0, 0) += noise_t();
        pose.tvec.at<double>(1, 0) += noise_t();
        pose.tvec.at<double>(2, 0) += noise_t();
        normal_distribution<> n_r(0, stddev_r);
        variate_generator<mt19937&, normal_distribution<> > noise_r(twister, n_r);
        pose.rvec.at<double>(0, 0) += noise_r();
        pose.rvec.at<double>(1, 0) += noise_r();
        pose.rvec.at<double>(2, 0) += noise_r();
    }

    cv::Mat randomOrientation(double angle) {
        Mat r = Mat::zeros(3, 1, CV_64FC1);
        r.at<double>(0, 0) = rand() - 0.5; 
        r.at<double>(1, 0) = rand() - 0.5; 
        r.at<double>(2, 0) = rand() - 0.5; 
        return angle * (r / norm(r));
    }

    PoseRT poseToPoseRT(const Pose & src) {
        PoseRT dst;
        Mat R;
        eigen2cv(src.t(), dst.tvec);
        eigen2cv(src.r(), R);
        Rodrigues(R, dst.rvec);
        dst.tvec.convertTo(dst.tvec, CV_64F);
        dst.rvec.convertTo(dst.rvec, CV_64F);
        return dst;
    }

    Pose poseRtToPose(const PoseRT & src) {
        Pose dst;
        Eigen::Vector3f t;
        cv2eigen(src.tvec, t);
        dst.setT(t);
        Mat R;
        Eigen::Matrix<double, 3, 3> Re;
        Rodrigues(src.rvec, R);
        cv2eigen(R, Re);
        dst.setR(R);
        return dst;
    }

    void writePose(const string & filename, const PoseRT & pose) {
        FileStorage f;
        f.open(filename, FileStorage::WRITE);
        f << PoseRT::YAML_NODE_NAME;
        pose.write(f);
        f.release();
    }

    void readPose(const string & filename, PoseRT & dst) {
        FileStorage f;
        f.open(filename, FileStorage::READ);
        dst.read(f[PoseRT::YAML_NODE_NAME]);
        f.release();
    }

    void modelToView(const Mat & mvtrans, const Mat & mvrot, const Mat & mpt, Mat & vpt) {
        vpt = mvrot * mpt + mvtrans;
    }

    void modelToView(const PoseRT & pose, const Point3d & mpt, Point3d & vpt) {
        vector<Point3d> op(1, mpt);
        Mat mop(op);
        Mat R;
        Rodrigues(pose.rvec, R);
        transform(mop, mop, R);
        op[0] += pose.tvec.at<Point3d>();
        vpt = op[0];
    }

    opencv_candidate::PoseRT translatePose(const opencv_candidate::PoseRT & src, const Mat & model_tvec) {
        PoseRT dst;
        dst.rvec = src.rvec.clone(); 
        Mat mvrot;
        Rodrigues(src.rvec, mvrot);
        modelToView(src.tvec, mvrot, model_tvec, dst.tvec);
        return dst;
    }

    opencv_candidate::PoseRT rotatePose(const opencv_candidate::PoseRT & src, const Mat & model_rvec) {
        Mat P;
        Rodrigues(src.rvec, P);
        Mat D;
        Rodrigues(model_rvec, D);
        PoseRT dst;
        Rodrigues(P * D, dst.rvec);
        dst.tvec = src.tvec.clone();
        return dst;
    }

    double angleBetweenVectors(const cv::Mat & u, const cv::Mat & v) {
        double c = u.dot(v) / (norm(u) * norm(v));
        // The calculation of cosine 'c' above is not always completely exact.
        // It is possible that c > 1 due to inexactness of floating-point
        // arithmetic. If min is omitted angleBetween(p, p) will produce NAN
        // instead of zero.
        return acos(min(1.0, c));
    }

    cv::Mat diffRotation(const cv::Mat & P, const cv::Mat & Q) {
        return P.inv() * Q;
    }

    double angleBetweenOrientations(const opencv_candidate::PoseRT & p,
                                    const opencv_candidate::PoseRT & q) {
        Mat P, Q;
        Rodrigues(p.rvec, P);
        Rodrigues(q.rvec, Q);
        Mat d;
        Rodrigues(P.inv() * Q, d);
        return norm(d);
    }

}

