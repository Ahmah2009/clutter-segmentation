/*
 * Author: Julius Adorf
 *
 * Provides helper methods for transforming model coordinates to camera
 * coordinates and vice versa, for reading and writing pose information, for
 * randomizing poses, translation and rotation. Basic data structures are
 * opencv_candidate::PoseRT, opencv_candidate::Pose, and cv::Mat. Note that
 * structs PoseRT and Pose are incompatible in many ways, especially in that
 * PoseRT uses double matrices and Pose uses float matrices. Make sure to
 * always access matrix elements of PoseRT.tvec and PoseRT.rvec via
 * Mat::at<double>.  If accessing as float, will result in garbage but not in a
 * runtime error. To make things worse, Pose matrices use float values. This
 * module tries to always use CV_64FC1 matrices for computations and favor
 * PoseRT objects over (conceptually equivalent) Pose objects.
 */

#ifndef _POSE_UTIL_H_
#define _POSE_UTIL_H_

#include "clutseg/gcc_diagnostic_disable.h"
    #include <boost/filesystem.hpp>
    #include <cv.h>
    #include <opencv_candidate/Camera.h>
    #include <opencv_candidate/PoseRT.h>
    #include <vector>
#include "clutseg/gcc_diagnostic_enable.h"

namespace clutseg {

    /** 
     * \brief A name plate and pose description for an object.
     *
     * A label is a just a name and a pose, like a name plate that can be
     * attached in a certain orientation to an object's origin. Can represent
     * ground truth.
     *
     * @see tod::Guess
     */
    struct Label {

        Label() : name("") {}
        Label(const std::string & name) : name(name) {}
        Label(const std::string & name, const opencv_candidate::PoseRT & pose) : name(name), pose(pose) {}

        std::string name;
        /** The pose of the object. Check pose.estimated whether it is available. */
        opencv_candidate::PoseRT pose; 

        void write(cv::FileStorage & fs) const;
        void read(const cv::FileNode & fn);

    };
    
    /**
     * \brief Ground truth for a single scene.
     *
     * A set of labels fully describes what objects can be seen where on the
     * scene. The ground truth is a set of labels. The result of recognition is
     * conceptually just a set of labels. For measuring performance, we can
     * take the ground truth label set and the recognized label set and compare
     * them. */
    struct LabelSet {

        std::vector<Label> labels;

        /** \brief Returns whether there is any object present in the scene */
        bool emptyScene() const { return labels.empty(); }
        /** \brief Returns whether the specific object is present in the scene */
        bool onScene(const std::string & name) const;
        /** \brief Returns the number of objects in the scenes, duplicates ignored. */
        int distinctLabelCount() const;
        /** \brief Returns the poses for a specified object (and duplicates). */
        std::vector<opencv_candidate::PoseRT> posesOf(const std::string & subject) const;

        /** \brief Writes the label set to a YAML sink. */
        void write(cv::FileStorage & fs) const;
        /** \brief Reads the label set from YAML source. */
        void read(const cv::FileNode & fn);

        const static std::string YAML_NODE_NAME;

    };

    /**
     * \brief Writes the label set to a YAML file.
     *
     * See clutseg/data/example.ground.yaml.
     */
    void writeLabelSet(const boost::filesystem::path & filename, const LabelSet & labelSet);

    /**
     * \brief Reads a label set from a YAML file.
     *
     * See clutseg/data/example.ground.yaml.
     */
    void readLabelSet(const boost::filesystem::path & filename, LabelSet & dst);

    /** \brief Finds the coordinates of the pixel where the object's origin
     * appears when projected onto the image plane. */
    cv::Point projectOrigin(const opencv_candidate::PoseRT & pose,
                            const opencv_candidate::Camera & camera);

    /** @deprecated     randomization of orientation is rather arbitrary */
    void randomizePose(opencv_candidate::PoseRT & pose, double stddev_t, double stddev_r);

    /** \brief Returns a rotation about a random rotation axis and the specified angle.
     * The returned 3x1 matrix is in axis-angle form. */
    cv::Mat randomOrientation(double angle);

    opencv_candidate::PoseRT poseToPoseRT(const opencv_candidate::Pose & src);

    opencv_candidate::Pose poseRtToPose(const opencv_candidate::PoseRT & src);

    /** \brief Writes a pose to a file. */
    void writePose(const boost::filesystem::path & filename, const opencv_candidate::PoseRT & pose);

    /** \brief Reads a pose from a file. Be careful that the file was previously generated using
     * clutseg::writePose. If it was written by Pose::write, the data might be messed up due to 
     * different data types (double <-> float) when read with this function. */
    void readPose(const boost::filesystem::path & filename, opencv_candidate::PoseRT & dst);


    /** 
     * \brief Converts the data types in legacy pose files written with Pose::write. F-up.
     *
     * Unfortunately, Pose::write and PoseRT::write create files that pretty
     * much look the same but are fully incompatible with each other in respect to
     * Pose::read, PoseRT::write and the member access of tvec and rvec via
     * Mat::at<double>. This method converts a YAML file that is known to have been
     * written by Pose::write to a file that can be read by PoseRT::read. If the
     * file contains multiple top level nodes, all of them are converted. Passing
     * the same filename as source and destination means converting in-place and is
     * supported. */
    void convertLegacyPoseFileToDouble(const boost::filesystem::path & src, const boost::filesystem::path & dst);

    /**
     * \brief Transforms model coordinates to camera coordinates.
     *
     * @param mvtrans   translation vector of the model-camera transformation
     * @param mvrot     rotation matrix of the model-camera transformation
     * @param mpt       model coordinates
     * @param vpt       camera coordinates  
     */
    void modelToView(const cv::Mat & mvtrans, const cv::Mat & mvrot, const cv::Mat & mpt, cv::Mat & vpt);

//TODO rename modelToView to modelToCamera

    /** \brief Transforms model coordinates to camera coordinates.
     *
     * @see modelToView
     */
    void modelToView(const opencv_candidate::PoseRT & pose, const cv::Point3d & mpt, cv::Point3d & vpt);

    opencv_candidate::PoseRT translatePose(const opencv_candidate::PoseRT & p, const cv::Mat & model_tvec);
   
    opencv_candidate::PoseRT rotatePose(const opencv_candidate::PoseRT & p, const cv::Mat & model_rvec);

    double angleBetweenVectors(const cv::Mat & u, const cv::Mat & v);

    /** Computes the rotation matrix D such that P * D = Q. Let inv(P) be the
     * inverse of P, then D = inv(P) * Q. Thus, rotation Q is decomposed into given
     * rotation P and the difference rotation this function computes. */
    cv::Mat diffRotation(const cv::Mat & P, const cv::Mat & Q);

    /**
     * \brief Computes the angle between two orientations (i.e. objects).
     *
     * The poses p and q define two coordinate systems with respect to the
     * camera coordinate system. The angle between these two coordinate systems
     * is computed. Consider the rotation matrices P and Q that correspond to
     * the proper rigid transformations p and q. The rotation matrix D is
     * computed, such that P * D = Q. The magnitude of rotation around the
     * rotation axis of D is returned.
     */
    double angle_between(const opencv_candidate::PoseRT & p,
                                    const opencv_candidate::PoseRT & q);

    /** \brief Computes the distance between the origin of p and the origin of q. */
    double dist_between(const opencv_candidate::PoseRT & p,
                                const opencv_candidate::PoseRT & q);

}

/* Alternatively
T * P = Q
T = Q * inv(P)
*/

#endif
