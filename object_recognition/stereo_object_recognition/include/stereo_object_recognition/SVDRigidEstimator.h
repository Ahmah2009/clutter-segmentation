#ifndef STEREO_OBJECT_RECOGNITION_SVD_RIGID_ESTIMATOR_H
#define STEREO_OBJECT_RECOGNITION_SVD_RIGID_ESTIMATOR_H

#include "stereo_object_recognition/RigidTransformationEstimator.h"

namespace stereo_object_recognition
{
/**
  Implementation of "Least-Squares Fitting of Two 3-D Point Sets" from K.S.ARUN, T.S.HUANG, S.D.BLOSTEIN
*/
  class SVDRigidEstimator : public RigidTransformationEstimator
  {
    public:
      SVDRigidEstimator( double epsilon = 1e-8 );
      virtual ~SVDRigidEstimator();
      virtual bool estimate( const std::vector<std::pair<cv::Point3d,cv::Point3d> >& point_correspondences,
                             cv::Mat& rotation_matrix, cv::Vec3d& translation_vector ) const;
      virtual RigidTransformationEstimator* clone() const;
    protected:
      static void svd( cv::Mat& mat, cv::Mat& S, cv::Mat& Ut, cv::Mat& V );
      double epsilon_;
  };
} //namespace stereo_object_recognition

#endif // STEREO_OBJECT_RECOGNITION_SVD_RIGID_ESTIMATOR_H