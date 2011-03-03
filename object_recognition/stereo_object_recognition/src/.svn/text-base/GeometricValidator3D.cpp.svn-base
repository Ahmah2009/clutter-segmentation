#include <vector>

#include "stereo_object_recognition/GeometricValidator3D.h"
#include "stereo_object_recognition/SVDRigidEstimator.h"

using namespace std;
using namespace cv;

namespace stereo_object_recognition
{
  GeometricValidator3D::GeometricValidator3D( double inlierThresholdSQR, double minPercentage, unsigned minInliers )
    : estimator_( SVDRigidEstimator(), inlierThresholdSQR, 0.999, 0.15, 1.05 )
    , min_inlier_percentage_( minPercentage )
    , min_inlier_count_( minInliers )
  {
  }

  GeometricValidator3D::~GeometricValidator3D(  )
  {
  }

  bool GeometricValidator3D::validate( const vector<pair<Point3d,Point3d> >& correspondences, Mat& rotationMatrix, Vec3d& translationVector, vector<unsigned>& matches ) const
  {
    unsigned minInlierFromPercentage = correspondences.size() * min_inlier_percentage_;
    if( correspondences.size() > std::max(minInlierFromPercentage, min_inlier_count_ ) && estimator_.estimate( correspondences, rotationMatrix, translationVector ) )
    {
      matches = estimator_.getInliers();

      if( std::max(minInlierFromPercentage, min_inlier_count_ ) <= (unsigned)matches.size() )
        return true;
      else
        return false;
    }
    return false;
  }
} //namespace stereo_object_recognition
