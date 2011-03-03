#ifndef STEREO_OBJECT_RECOGNITION_RANSAC_RIFIG_ESTIMATOR_H
#define STEREO_OBJECT_RECOGNITION_RANSAC_RIFIG_ESTIMATOR_H

#include "stereo_object_recognition/RigidTransformationEstimator.h"

namespace stereo_object_recognition
{
  class RANSACRigidTransformationEstimator : public RigidTransformationEstimator
  {
    public:
      RANSACRigidTransformationEstimator( const RigidTransformationEstimator& minimal_solution_estimator,
                                          double inlier_distance_threshold_SQR, double min_valid_set_probability = 0.999,
                                          double inlier_probability = 0.9, double maximum_scale_factor = 1.05 );

      RANSACRigidTransformationEstimator( const RigidTransformationEstimator& minimal_solution_estimator,
                                          double inlier_distance_threshold_SQR, unsigned minimum_draws,
                                          double maximum_scale_factor = 1.05 );

      RANSACRigidTransformationEstimator( const RANSACRigidTransformationEstimator& );
      virtual ~RANSACRigidTransformationEstimator();
      virtual bool estimate( const std::vector<std::pair<cv::Point3d,cv::Point3d> >& point_correspondences,
                             cv::Mat& rotation_matrix, cv::Vec3d& translation_vector ) const;
      virtual RigidTransformationEstimator* clone() const;
      const std::vector<unsigned>& getInliers() const;
    private:
      
    protected:
      //const RigidTransformationEstimator& m_minSolEstimator;
      RigidTransformationEstimator* minimal_solution_estimator_;
      double inlier_distance_threshold_SQR_;
      unsigned mininum_draws_;
      double maximum_scale_factor_;
      static const int MINIMUM_DRAWS = 200;
      static const int MAXIMUM_DRAWS = 10000;
      mutable std::vector<unsigned> inlier_indices_;
  };

  inline const std::vector<unsigned>& RANSACRigidTransformationEstimator::getInliers() const
  {
      return inlier_indices_;
  }
} //namespace stereo_object_recognition

#endif // STEREO_OBJECT_RECOGNITION_RANSAC_RIFIG_ESTIMATOR_H