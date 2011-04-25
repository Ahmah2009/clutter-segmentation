/*
 * Author: Julius Adorf
 */

#include <cv.h>
#include <opencv_candidate/Camera.h>
#include <opencv_candidate/PoseRT.h>
#include <opencv/cxeigen.hpp>
#include <tod/detecting/GuessGenerator.h>
#include <tod/core/TrainingBase.h>
#include <tod/core/Features2d.h>
#include <tod/detecting/Matcher.h>

using namespace cv;
using namespace opencv_candidate;
using namespace tod;

namespace clutseg {

    /** \brief Draws keypoints on top of a canvas.
     * 
     * Delegates drawing to OpenCV but makes sure to set default values that
     * are consistently used in package clutseg.
     *
     * @param canvas    Output image. Any valid non-empty matrix.
     * @param keypoints
     * @param color
     */
    void drawKeypoints(Mat & canvas, const vector<KeyPoint> & keypoints,
                        const Scalar & color = Scalar(0, 0, 255));

    /** \brief Draws guess inliers on top of a canvas. */
    void drawInliers(Mat & canvas, const Guess & guess,
                        const Scalar & color = Scalar(0, 255, 0));

    /** \brief Draws a given pose.
     *
     * The pose is projected on the canvas using the camera information. The
     * result shows a coordinate system that visualizes the pose on the image.
     * Pose will be drawn on top of existing canvas content.
     *
     * This function has been derived from TOD fiducial package, but has been
     * extended to support drawing the axes in customizable colors and with
     * custom axis labels. */
    void drawPose(Mat & canvas, const PoseRT & pose, const opencv_candidate::Camera & camera,
                  const Scalar & colorX = Scalar(255, 0,0),
                  const Scalar & colorY = Scalar(0, 255, 0),
                  const Scalar & colorZ = Scalar(0, 0, 255),
                  const string & labelX = "X",
                  const string & labelY = "Y",
                  const string & labelZ = "Z");

    void drawPose(Mat & canvas, const Pose & pose,
                  const opencv_candidate::Camera & camera,
                  const Scalar & colorX = Scalar(255, 0,0),
                  const Scalar & colorY = Scalar(0, 255, 0),
                  const Scalar & colorZ = Scalar(0, 0, 255),
                  const string & labelX = "X",
                  const string & labelY = "Y",
                  const string & labelZ = "Z");

    /** \brief Draws guesses onto the canvas. The inliers will be drawn as well
     * as the guessed poses and the labels. If ground truth is available, the true
     * poses for the tagged subject will also be visualized. For every guess, the
     * same color will be used for the label, pose and inliers. Ground truth will
     * be drawn in shades of gray for all guesses but those will be easily
     * distinguishable anyway.
     */
    void drawGuesses(Mat & canvas, const vector<Guess> & guesses,
                        const Camera & camera, const vector<PoseRT> & ground_poses);

    /** \brief Convenience method, just delegates to drawGuesses */
    void drawGuess(Mat & canvas, const Guess & guess,
                    const Camera & camera, const PoseRT ground_pose);

    /** \brief Prints a multiline text onto the canvas and returns a bounding
     * rectangle. 
     */
    Rect drawText(Mat& canvas, const vector<string> & lines,
                    const Point & topleft, int fontFace, double fontScale,
                    int thickness, const Scalar & color);

    // TODO: draw matches for individual objects

    /** \brief Draws matches from query image to all training images and puts
     * them together in a big collage. */
    void drawAllMatches(Mat & canvas, const TrainingBase & base,
                            const Ptr<Matcher> matcher, const Mat& testImage,
                            const KeypointVector & testKeypoints,
                            const string & baseDirectory);

}

