/**
 * Author: Julius Adorf
 *
 * Provides helper methods for transforming model coordinates to camera
 * coordinates and vice versa, for reading and writing pose information, for
 * randomizing poses, translation and rotation. Basic data structures are
 * opencv_candidate::PoseRT, opencv_candidate::Pose, and cv::Mat. There seem to
 * be some issues with type confusion, depending on whether constructors are
 * used or poses are read from file. This module tries to always use CV_64FC1
 * matrices for computations, but this policy has not been implemented in each
 * method yet, so watch out! 
 */

#ifndef _POSE_UTIL_H_
#define _POSE_UTIL_H_

#include <boost/filesystem.hpp>
#include <cv.h>
#include <opencv_candidate/Camera.h>
#include <opencv_candidate/PoseRT.h>

namespace clutseg {

    cv::Point projectOrigin(const opencv_candidate::PoseRT & pose, const opencv_candidate::Camera & camera);

    /** @deprecated     randomization of orientation is rather arbitrarily */
    void randomizePose(opencv_candidate::PoseRT & pose, double stddev_t, double stddev_r);

    /** Returns an orientation with random rotation axis and the specified
     * angle in axis-angle representation. */
    cv::Mat randomOrientation(double angle);

    opencv_candidate::PoseRT poseToPoseRT(const opencv_candidate::Pose & src);

    opencv_candidate::Pose poseRtToPose(const opencv_candidate::PoseRT & src);

    void writePose(const boost::filesystem::path & filename, const opencv_candidate::PoseRT & pose);

    void readPose(const boost::filesystem::path & filename, opencv_candidate::PoseRT & dst);

    void modelToView(const cv::Mat & mvtrans, const cv::Mat & mvrot, const cv::Mat & mpt, cv::Mat & vpt);

    void modelToView(const opencv_candidate::PoseRT & pose, const cv::Point3d & mpt, cv::Point3d & vpt);

    /** Translates a given pose. The translation vector 'model_tvec' is given
     * in model coordinates.*/
    opencv_candidate::PoseRT translatePose(const opencv_candidate::PoseRT & p, const cv::Mat & model_tvec);
   
    /** Rotates a given pose. Let's say P is the orientation of pose 'p', and
     * the rotation matrix D is equivalent to axis-angle representation
     * 'model_rvec'. Then, a new orientation Q is computed by Q = P * D.
     * Conceptually, the invariant Q = P * diffRotation(P, Q) holds. */
    opencv_candidate::PoseRT rotatePose(const opencv_candidate::PoseRT & p, const cv::Mat & model_rvec);

    double angleBetweenVectors(const cv::Mat & u, const cv::Mat & v);

    /** Computes the rotation matrix D such that P * D = Q. Let inv(P) be the
     * inverse of P, then D = inv(P) * Q. Thus, rotation Q is decomposed into given
     * rotation P and the difference rotation this function computes. */
    cv::Mat diffRotation(const cv::Mat & P, const cv::Mat & Q);

    /** Computes the angle between two orientations given by poses p and q,
     * with rotation matrices P and Q. Function diffRotation computes the rotation
     * matrix D such that P * D = Q. This difference D is a rotation around a
     * certain axis about a certain angle. The angle is returned by this function.
     */
    double angle_between(const opencv_candidate::PoseRT & p,
                                    const opencv_candidate::PoseRT & q);

    /** Computes the distance between the origin of p and the origin of q. */
    double dist_between(const opencv_candidate::PoseRT & p,
                                const opencv_candidate::PoseRT & q);

}

/* Alternatively
T * P = Q
T = Q * inv(P)
*/

#endif
