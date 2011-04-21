/*
 * Author: Julius Adorf
 */

#include <cv.h>
#include <opencv_candidate/PoseRT.h>
#include <opencv_candidate/Camera.h>

using namespace cv;
using namespace opencv_candidate;

namespace clutseg {

    // TODO: use template or remove one of these methods
    /** Draws a given pose. The pose is projected on the canvas using
     * the camera information. The result shows a coordinate system that
     * visualizes the pose on the image. */
    void drawPose(Mat & canvas, const PoseRT & pose, const opencv_candidate::Camera & camera,
                  const Scalar & colorX = Scalar(255, 0,0),
                  const Scalar & colorY = Scalar(0, 255, 0),
                  const Scalar & colorZ = Scalar(0, 0, 255),
                  const string & labelX = "X",
                  const string & labelY = "Y",
                  const string & labelZ = "Z");

    void drawPose(Mat & canvas, const Pose & pose, const Mat & image,
                  const opencv_candidate::Camera & camera,
                  const Scalar & colorX = Scalar(255, 0,0),
                  const Scalar & colorY = Scalar(0, 255, 0),
                  const Scalar & colorZ = Scalar(0, 0, 255),
                  const string & labelX = "X",
                  const string & labelY = "Y",
                  const string & labelZ = "Z");

    void poseToPoseRT(const Pose & src, PoseRT & dst);

    void poseRtToPose(const PoseRT & src, Pose & dst);

    // TODO: use template or remove one of these methods
    void writePose(const string & filename, const PoseRT & pose);

    void readPose(const string & filename, PoseRT & dst);

    void writePose(const string & filename, const Pose & pose);

    void readPose(const string & filename, Pose & dst);

}

