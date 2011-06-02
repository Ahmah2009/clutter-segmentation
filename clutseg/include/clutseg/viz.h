/*
 * Author: Julius Adorf
 */

#ifndef _VIZ_H_
#define _VIZ_H_

#include "clutseg/ground.h"

#include <cv.h>
#include <opencv_candidate/Camera.h>
#include <opencv_candidate/PoseRT.h>
#include <tod/detecting/GuessGenerator.h>
#include <tod/core/TrainingBase.h>
#include <tod/core/Features2d.h>
#include <tod/detecting/Matcher.h>

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
    void drawKeypoints(cv::Mat & canvas, const std::vector<cv::KeyPoint> & keypoints,
                        const cv::Scalar & color = cv::Scalar(0, 0, 255));

    /** \brief Draws guess inliers on top of a canvas. */
    void drawInliers(cv::Mat & canvas, const tod::Guess & guess,
                        const cv::Scalar & color = cv::Scalar(0, 255, 0));

    /** \brief Draws a given pose.
     *
     * The pose is projected on the canvas using the camera information. The
     * result shows a coordinate system that visualizes the pose on the image.
     * Pose will be drawn on top of existing canvas content.
     *
     * This function has been derived from TOD fiducial package, but has been
     * extended to support drawing the axes in customizable colors and with
     * custom axis labels. */
    void drawPose(cv::Mat & canvas, const opencv_candidate::PoseRT & pose, const opencv_candidate::Camera & camera,
                  const cv::Scalar & colorX = cv::Scalar(255, 0,0),
                  const cv::Scalar & colorY = cv::Scalar(0, 255, 0),
                  const cv::Scalar & colorZ = cv::Scalar(0, 0, 255),
                  const std::string & labelX = "X",
                  const std::string & labelY = "Y",
                  const std::string & labelZ = "Z");

    /** \brief Draws guesses onto the canvas. The inliers will be drawn as well
     * as the guessed poses and the labels. If ground truth is available, the true
     * poses for the tagged subject will also be visualized. For every guess, the
     * same color will be used for the label, pose and inliers. Ground truth will
     * be drawn in shades of gray for all guesses but those will be easily
     * distinguishable anyway.
     */
    void drawGuesses(cv::Mat & canvas, const std::vector<tod::Guess> & guesses,
                        const opencv_candidate::Camera & camera,
                        const std::vector<opencv_candidate::PoseRT> & ground_poses);

    void drawGuesses(cv::Mat & canvas, const std::vector<tod::Guess> & guesses,
                        const opencv_candidate::Camera & camera);

    /** \brief Convenience method, just delegates to drawGuesses */
    void drawGuess(cv::Mat & canvas, const tod::Guess & guess,
                    const opencv_candidate::Camera & camera, const opencv_candidate::PoseRT ground_pose);

    /** \brief Prints a multiline text onto the canvas and returns a bounding
     * rectangle. 
     */
    cv::Rect drawText(cv::Mat& canvas, const std::vector<std::string> & lines,
                    const cv::Point & topleft, int fontFace, double fontScale,
                    int thickness, const cv::Scalar & color);

    /** Draws a label at the projected origin of a pose. */
    void drawLabelAtOrigin(cv::Mat & canvas, const opencv_candidate::PoseRT & pose, const opencv_candidate::Camera & camera,
                            const std::string & label, const cv::Scalar & color);
    // TODO: draw matches for individual objects

    /** \brief Draws matches from query image to all training images and puts
     * them together in a big collage. */
    void drawAllMatches(cv::Mat & canvas, const tod::TrainingBase & base,
                            const cv::Ptr<tod::Matcher> matcher, const cv::Mat& testImage,
                            const tod::KeypointVector & testKeypoints,
                            const std::string & baseDirectory);

    /** Draws ground truth poses and labels */
    void drawGroundTruth(cv::Mat & canvas, const clutseg::GroundTruth & groundTruth,
                            const opencv_candidate::Camera & camera);
}

#endif
