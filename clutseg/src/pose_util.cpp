/*
 * Author: Julius Adorf
 */

#include "clutseg/pose_util.h"

#include <fiducial/fiducial.h>
#include <opencv/cxeigen.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random.hpp>
#include <boost/foreach.hpp>

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
        //return Point(static_cast<int>(ip[0].x), static_cast<int>(ip[0].y));
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

    void poseToPoseRT(const Pose & src, PoseRT & dst) {
        Mat R;
        eigen2cv(src.t(), dst.tvec);
        eigen2cv(src.r(), R);
        Rodrigues(R, dst.rvec);
        dst.tvec.convertTo(dst.tvec, CV_64F);
        dst.rvec.convertTo(dst.rvec, CV_64F);
    }

    void poseRtToPose(const PoseRT & src, Pose & dst) {
        Eigen::Vector3f t;
        cv2eigen(src.tvec, t);
        dst.setT(t);
        Mat R;
        Eigen::Matrix<double, 3, 3> Re;
        Rodrigues(src.rvec, R);
        cv2eigen(R, Re);
        dst.setR(R);
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

    // TODO: use template or remove one of these methods
    void writePose(const string & filename, const Pose & pose) {
        FileStorage f;
        f.open(filename, FileStorage::WRITE);
        f << PoseRT::YAML_NODE_NAME;
        pose.write(f);
        f.release();
    }

    void readPose(const string & filename, Pose & dst) {
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

    void translatePose(const opencv_candidate::PoseRT & src, const Mat & model_tvec, opencv_candidate::PoseRT & dst) {
        dst.rvec = src.rvec.clone(); 
        Mat mvrot;
        Rodrigues(src.rvec, mvrot);
        modelToView(src.tvec, mvrot, model_tvec, dst.tvec);
    }

}

