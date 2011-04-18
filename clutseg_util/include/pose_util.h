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
    void drawPose(const PoseRT & pose, const Mat & image, const Camera & camera, Mat & canvas);

    void drawPose(const Pose & pose, const Mat & image, const Camera & camera, Mat & canvas);

    void poseToPoseRT(const Pose & src, PoseRT & dst);

    void poseRtToPose(const PoseRT & src, Pose & dst);

    // TODO: use template or remove one of these methods
    void writePose(const string & filename, const PoseRT & pose);

    void readPose(const string & filename, PoseRT & dst);

    void writePose(const string & filename, const Pose & pose);

    void readPose(const string & filename, Pose & dst);

}

