/*
 * Author: Julius Adorf
 */

#include "viz.h"

using namespace cv;
using namespace opencv_candidate;

namespace clutseg {

    void drawKeypoints(Mat & canvas, const vector<KeyPoint> & keypoints,
                        const Scalar & color) { 
        // Use OpenCV's drawKeypoints method DrawMatchesFlags::DRAW_OVER_OUTIMG
        // specifies that keypoints are simply drawn onto existing content of
        // the output canvas. The first image parameter of drawKeypoints is
        // then not used by drawKeypoints. See also
        // https://code.ros.org/svn/opencv/trunk/opencv/modules/features2d/src/draw.cpp
        cv::drawKeypoints(canvas, keypoints, canvas, color,
                            DrawMatchesFlags::DRAW_OVER_OUTIMG);
    }

    void drawInliers(Mat & canvas, const Guess & guess, const Scalar & color) {
        vector<KeyPoint> kpts;
        for (size_t i = 0; i < guess.inliers.size(); i++) {
            Point2f kp = guess.image_points_[guess.inliers[i]];
            kpts.push_back(KeyPoint(kp, 0));
        }
        clutseg::drawKeypoints(canvas, kpts, color);
    }

    void drawPose(Mat & canvas, const PoseRT & pose, const Camera & camera,
                const Scalar & colorX, const Scalar & colorY, const Scalar & colorZ,
                const string & labelX, const string & labelY, const string & labelZ) {
        // see fiducial::PoseDrawer(canvas, camera.K, pose);
        // most of the code has been copied from that function
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
        /* see question on answers.ros.org
        string scaleText = "scale 0.25 meters";
        int baseline = 0;
        Size sz = getTextSize(scaleText, CV_FONT_HERSHEY_SIMPLEX, 1, 1, &baseline);
        rectangle(canvas, Point(10, 30 + 5), Point(10, 30) + Point(sz.width, -sz.height - 5), Scalar::all(0), -1);
        */
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

}

