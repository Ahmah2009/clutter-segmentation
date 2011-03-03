#ifndef STEREO_OBJECT_RECOGNITION_RIGID_ESTIMATOR_H
#define STEREO_OBJECT_RECOGNITION_RIGID_ESTIMATOR_H

#include <opencv2/core/core.hpp>
#include <vector>
#include <utility>

namespace stereo_object_recognition
{
  /**
   * \author Suat Gedikli
   * \date 5.Aug 2010
   * \brief base class for a rigid tranformation estimator
   */
  class RigidTransformationEstimator
  {
    public:
      virtual ~RigidTransformationEstimator()
      {};
      
      virtual bool estimate( const std::vector<std::pair<cv::Point3d,cv::Point3d> >& point_correspondences,
                             cv::Mat& rotation_matrix, cv::Vec3d& translation_vector ) const = 0;

      virtual RigidTransformationEstimator* clone() const = 0;
  };
} //namespace stereo_object_recognition

#endif //STEREO_OBJECT_RECOGNITION_RIGID_ESTIMATOR_H