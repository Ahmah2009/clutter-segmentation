#ifndef STEREO_OBJECT_RECOGNITION_GEOMETRIC_VALIDATOR_3D_H
#define STEREO_OBJECT_RECOGNITION_GEOMETRIC_VALIDATOR_3D_H

#include <opencv2/core/core.hpp>
#include <vector>
#include <utility>
#include "stereo_object_recognition/RANSACRigidTransformationEstimator.h"

namespace stereo_object_recognition
{
  class GeometricValidator3D
  {
    public:
    /**
     * \author Suat Gedikli
     * \date 5. Aug 2010
     * \brief Constructor
     * \param[in] inlier_threshold_SQR the maximum squared euclidean distance for a valid correspondence
     * \param[in] min_inlier_percentage minimum precentage of inlying correspondences for a valid rigid transformation
     * \param[in] min_inlierCount minimum number of inlying correspondences for a valid rigid transformation
     */
      GeometricValidator3D( double inlier_threshold_SQR, double min_inlier_percentage = 0.15, unsigned min_inlierCount = 50 );

    /**
     * \author Suat Gedikli
     * \date 5.Aug 2010
     * \brief Destructor
     */
      virtual ~GeometricValidator3D();

    /**
     * \author Suat Gedikli
     * \date 5. Aug 2010
     * \brief estimates the rigid transformation from the first point-cloud to the second point-cloud given by correspondences
     * \param[in] correspondences corresponding points
     * \param[out] rotation_matrix rotation matrix of the rigid transformation, if found
     * \param[out] translation_vector translation of the rigid transformation, if found
     * \param[out] array of inidices of inliers (matching correspondences)
     * \return true if a ridig transformation between both point clouds could be found. false otherwise
     */
      virtual bool validate( const std::vector<std::pair<cv::Point3d,cv::Point3d> >& correspondences,
                             cv::Mat& rotation_matrix, cv::Vec3d& translation_vector,
                             std::vector<unsigned>& matches ) const;

    protected:
      // the used rigid transformation estimator
      RANSACRigidTransformationEstimator estimator_;
      double min_inlier_percentage_;
      unsigned min_inlier_count_;
  };
} //namespace stereo_object_recognition

#endif // STEREO_OBJECT_RECOGNITION_GEOMETRIC_VALIDATOR_3D_H
