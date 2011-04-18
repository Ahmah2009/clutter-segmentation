/*
 * Author: Julius Adorf
 */

#include "pose_util.h"

#include <fiducial/fiducial.h>
#include <opencv/cxeigen.hpp>
#include <opencv2/calib3d/calib3d.hpp>

using namespace fiducial;
using namespace cv;
using namespace opencv_candidate;

namespace clutseg {

    void drawPose(const PoseRT & pose, const Mat & image, const Camera & camera, Mat & canvas) {
        PoseDrawer(canvas, camera.K, pose);
    }

    void drawPose(const Pose & pose, const Mat & image, const Camera & camera, Mat & canvas) {
        PoseRT posert;
        Mat R;
        eigen2cv(pose.t(), posert.tvec);
        eigen2cv(pose.r(), R);
        Rodrigues(R, posert.rvec);
        drawPose(posert, image, camera, canvas);
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

}

