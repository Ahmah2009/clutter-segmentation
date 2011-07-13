/*
 * Author: Julius Adorf
 */

#ifndef _VIZ_H_
#define _VIZ_H_

#include "clutseg/ground.h"

#include "clutseg/gcc_diagnostic_disable.h"
    #include <cv.h>
    #include <opencv_candidate/Camera.h>
    #include <opencv_candidate/PoseRT.h>
    #include <tod/detecting/GuessGenerator.h>
    #include <tod/core/TrainingBase.h>
    #include <tod/core/Features2d.h>
    #include <tod/detecting/Matcher.h>
#include "clutseg/gcc_diagnostic_enable.h"

namespace clutseg {

    /**
     * \brief Draws keypoints on top onto a canvas. The canvas must be initialized.
     */
    void drawKeypoints(cv::Mat & canvas, const std::vector<cv::KeyPoint> & keypoints,
                        const cv::Scalar & color = cv::Scalar(0, 0, 255));

    /** 
     * \brief Draws the inliers of a guess onto a canvas. The canvas must be initialized.
     */
    void drawInliers(cv::Mat & canvas, const tod::Guess & guess,
                        const cv::Scalar & color = cv::Scalar(0, 255, 0));

    /**
     * \brief Draws a given pose (i.e. the object coordinate system) onto a canvas.
     *
     * The canvas must be initialized. The axes of the object coordinate system
     * are projected on the canvas using the camera information.
     *
     * This function has been derived from TOD fiducial package, but has been
     * extended to support drawing the axes in customizable colors and with
     * custom axis labels.
     */
    void drawPose(cv::Mat & canvas, const opencv_candidate::PoseRT & pose, const opencv_candidate::Camera & camera,
                  const cv::Scalar & colorX = cv::Scalar(255, 0,0),
                  const cv::Scalar & colorY = cv::Scalar(0, 255, 0),
                  const cv::Scalar & colorZ = cv::Scalar(0, 0, 255),
                  const std::string & labelX = "X",
                  const std::string & labelY = "Y",
                  const std::string & labelZ = "Z");

    /**
     * \brief Draws guesses onto a canvas.
     *  
     * The canvas must be initialized. Draws the inliers, the guessed pose, the
     * label, and the ground truth pose (if available) for each guess. Each
     * guess is assigned a color. Only the ground truth is always coloured in
     * shades of grey.
     */
    void drawGuesses(cv::Mat & canvas, const std::vector<tod::Guess> & guesses,
                        const opencv_candidate::Camera & camera,
                        const std::vector<opencv_candidate::PoseRT> & ground_poses);

    /** \brief Draws guesses onto a canvas. See clutseg::drawGuesses. */
    void drawGuesses(cv::Mat & canvas, const std::vector<tod::Guess> & guesses,
                        const opencv_candidate::Camera & camera);

    /** \brief Draws a guess onto a canvas. See clutseg::drawGuesses. */
    void drawGuess(cv::Mat & canvas, const tod::Guess & guess,
                    const opencv_candidate::Camera & camera, const opencv_candidate::PoseRT ground_pose);

    /**
     * \brief Draws a multiline text onto the canvas and returns a bounding
     * rectangle. 
     *
     * The canvas must be initialized. Each vector item contains one line of
     * the text. The bounding rectangle of the resulting text is computed and
     * returned.
     */
    cv::Rect drawText(cv::Mat& canvas, const std::vector<std::string> & lines,
                    const cv::Point & topleft, int fontFace, double fontScale,
                    int thickness, const cv::Scalar & color);

    /**
     * \brief Draws a label onto the canvas at the projected origin of a pose.
     *
     * The canvas must be initialized.
     */
    void drawLabelAtOrigin(cv::Mat & canvas, const opencv_candidate::PoseRT & pose, const opencv_candidate::Camera & camera,
                            const std::string & label, const cv::Scalar & color);

    /**
     * \brief Draws matches between the query image and all training images
     * which have more than a minimum number of matches and draws them onto the
     * canvas.
     *
     * The canvas must be initialized. The canvas is arranged as a big collage.
     */
    void drawAllMatches(cv::Mat & canvas, const tod::TrainingBase & base,
                            const cv::Ptr<tod::Matcher> matcher, const cv::Mat& testImage,
                            const tod::KeypointVector & testKeypoints,
                            const std::string & baseDirectory);

    /**
     * \brief Draws ground truth poses and labels onto the canvas.
     * 
     * The canvas must be initialized.
     */
    void drawGroundTruth(cv::Mat & canvas, const clutseg::LabelSet & groundTruth,
                            const opencv_candidate::Camera & camera);


    /**
     * \brief The three coordinate planes of a Cartesian coordinate system.
     * X-Y-plane, X-Z-plane, and Z-X-plane.
     */
    enum CoordinatePlane {
        XY, YZ, ZX 
    };

    /**
     * \brief Draws a histogram of a point cloud by projecting it orthogonally onto a
     * coordinate plane.
     *
     * @param hist      the destination image, a histogram
     * @param cloud     the point cloud
     * @param plane     one of the three Cartesian coordinate planes
     * @param a_min     A-coordinates less than a_min will be ignored 
     * @param a_max     A-coordinates greater than a_max will be ignored
     * @param a_w       width of bins in axis direction of A
     * @param b_min     B-coordinates less than b_min will be ignored
     * @param b_max     B-coordinates greater than b_max will be ignored
     * @param b_w       width of bins in axis direction of B
     * @param draw_axes whether to draw the axis onto the destination image
     */
    void drawCoordinateHist(cv::Mat & hist,
                                    const pcl::PointCloud<pcl::PointXYZ> cloud,
                                    const CoordinatePlane & plane,
                                    float a_min, float a_max, float a_w,
                                    float b_min, float b_max, float b_w,
                                    bool draw_axes = false);

}

#endif
