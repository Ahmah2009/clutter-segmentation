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

    // This function has been derived from TOD
    // fiducial package, but has been extended to support drawing the axes in
    // customizable colors. */
    void drawPose(Mat & canvas, const PoseRT & pose, const Camera & camera,
                const Scalar & colorX, const Scalar & colorY, const Scalar & colorZ,
                const string & labelX, const string & labelY, const string & labelZ) {
        // PoseDrawer(canvas, camera.K, pose);
        Point3f z(0, 0, 0.25);
        Point3f x(0.25, 0, 0);
        Point3f y(0, 0.25, 0);
        Point3f o(0, 0, 0);
        vector<Point3f> op(4);
        op[1] = x, op[2] = y, op[3] = z, op[0] = o;
        vector<Point2f> ip;
        projectPoints(Mat(op), pose.rvec, pose.tvec, camera.K, Mat(), ip);

        vector<Scalar> c(4); //colors
        c[0] = Scalar(255, 255, 255);
        c[1] = colorX; //Scalar(255, 0, 0);//x
        c[2] = colorY; //Scalar(0, 255, 0);//y
        c[3] = colorZ; //Scalar(0, 0, 255);//z
        line(canvas, ip[0], ip[1], c[1]);
        line(canvas, ip[0], ip[2], c[2]);
        line(canvas, ip[0], ip[3], c[3]);
        string scaleText = "scale 0.25 meters";
        int baseline = 0;
        Size sz = getTextSize(scaleText, CV_FONT_HERSHEY_SIMPLEX, 1, 1, &baseline);
        rectangle(canvas, Point(10, 30 + 5), Point(10, 30) + Point(sz.width, -sz.height - 5), Scalar::all(0), -1);
        putText(canvas, scaleText, Point(10, 30), CV_FONT_HERSHEY_SIMPLEX, 1.0, c[0], 1, CV_AA, false);
        putText(canvas, labelZ, ip[3], CV_FONT_HERSHEY_SIMPLEX, 0.5, c[3], 1, CV_AA, false);
        putText(canvas, labelY, ip[2], CV_FONT_HERSHEY_SIMPLEX, 0.5, c[2], 1, CV_AA, false);
        putText(canvas, labelX, ip[1], CV_FONT_HERSHEY_SIMPLEX, 0.5, c[1], 1, CV_AA, false);
    }

    void drawPose(Mat & canvas, const Pose & pose, const Camera & camera,
                const Scalar & colorX, const Scalar & colorY, const Scalar & colorZ,
                const string & labelX, const string & labelY, const string & labelZ) {
        PoseRT posert;
        Mat R;
        eigen2cv(pose.t(), posert.tvec);
        eigen2cv(pose.r(), R);
        Rodrigues(R, posert.rvec);
        drawPose(canvas, posert, camera, colorX, colorY, colorZ, labelX, labelY, labelZ);
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

