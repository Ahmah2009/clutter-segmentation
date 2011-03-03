#include "stereo_object_recognition/RANSACRigidTransformationEstimator.h"
#include <algorithm>
#include <iostream>

using namespace std;
using namespace cv;

namespace stereo_object_recognition
{
  RANSACRigidTransformationEstimator::RANSACRigidTransformationEstimator( const RigidTransformationEstimator& minimalSolutionEstimator,
                                                                          double inlierThresholdSQR, double minProb,
                                                                          double inlierProb, double maximum_scale_factor )
  : minimal_solution_estimator_( minimalSolutionEstimator.clone() )
  , inlier_distance_threshold_SQR_( inlierThresholdSQR )
  , maximum_scale_factor_( maximum_scale_factor )
  {

    double dirtySetProb = 1.0 - pow( inlierProb, 3 );
    double minimumDraws = log( 1.0 - minProb ) / log( dirtySetProb );
    mininum_draws_ = min( max( (unsigned) minimumDraws + 1, (unsigned) MINIMUM_DRAWS ), (unsigned) MAXIMUM_DRAWS );
  }

  RANSACRigidTransformationEstimator::RANSACRigidTransformationEstimator( const RigidTransformationEstimator& minimal_solution_estimator,
                                                                          double inlier_distance_threshold_SQR,
                                                                          unsigned minimum_draws,
                                                                          double maximum_scale_factor )
  : minimal_solution_estimator_( minimal_solution_estimator.clone() )
  , inlier_distance_threshold_SQR_( inlier_distance_threshold_SQR )
  , mininum_draws_( minimum_draws )
  , maximum_scale_factor_( maximum_scale_factor )
  {
  }

   RANSACRigidTransformationEstimator::RANSACRigidTransformationEstimator( const RANSACRigidTransformationEstimator& ransacRigidEstimator )
  : minimal_solution_estimator_( ransacRigidEstimator.minimal_solution_estimator_->clone() )
  , inlier_distance_threshold_SQR_( ransacRigidEstimator.inlier_distance_threshold_SQR_ )
  , mininum_draws_( ransacRigidEstimator.mininum_draws_ )
  , maximum_scale_factor_( ransacRigidEstimator.maximum_scale_factor_ )
  {
  }

  RANSACRigidTransformationEstimator::~RANSACRigidTransformationEstimator()
  {
    delete minimal_solution_estimator_;
  }

  bool RANSACRigidTransformationEstimator::estimate( const vector<pair<Point3d,Point3d> >& pointCorrespondences,
                                                     Mat& rotMat, Vec3d& translation ) const
  {
    if( pointCorrespondences.size() < 3 )
      return false;

    bool found = false;
    unsigned drawCount = 0;
    vector<unsigned>* bestInliers = new vector<unsigned>();
    
    #pragma omp parallel shared( drawCount, bestInliers, found )
    {
    vector<unsigned>* tempInliers = new vector<unsigned>();
    Mat rotMat( 3, 3, CV_64FC1 );
    Vec3d translation;
    vector<pair<Point3d,Point3d> > subSet( 3 );
    CvRNG rng = cvRNG();

    double* row[3];
    row[0] = rotMat.ptr<double>( 0 );
    row[1] = rotMat.ptr<double>( 1 );
    row[2] = rotMat.ptr<double>( 2 );

    do {
      // usually continue leads to evaluation of while condition => while(++drawCount < min )
      // should work. But since we use openmp ++drawCount has to be atomic => standalone =>
      // while condition is without ++ => continue does not increment automatically the drawcount!
      // therefore we increment manually at the beginning of the loop
      #pragma omp atomic
      ++drawCount;

      // get randomly a minimal subset of pointcorrs
      subSet[0] = pointCorrespondences[ cvRandInt( &rng ) % pointCorrespondences.size() ];
      subSet[1] = pointCorrespondences[ cvRandInt( &rng ) % pointCorrespondences.size() ];
      subSet[2] = pointCorrespondences[ cvRandInt( &rng ) % pointCorrespondences.size() ];

      double lenA1 = norm((subSet[0].first - subSet[1].first));
      double lenA2 = norm((subSet[0].first - subSet[2].first));
      double lenA3 = norm((subSet[2].first - subSet[1].first));
      double lenB1 = norm((subSet[0].second - subSet[1].second));
      double lenB2 = norm((subSet[0].second - subSet[2].second));
      double lenB3 = norm((subSet[2].second - subSet[1].second));

      // if the triangle-side is smaller than 0.5xerror-threshold => bad conditioned!
      double minSide = min( min( min( lenA1, lenB1 ), min( lenA2, lenB2 )),min( lenA3, lenB3 ));
      if( minSide < (inlier_distance_threshold_SQR_ * 0.25) )
        continue;

      // check wheter the triangle sides have almost equal length
      if( min( lenA1, lenB1 ) * maximum_scale_factor_ < max( lenA1, lenB1 ) ||
          min( lenA2, lenB2 ) * maximum_scale_factor_ < max( lenA2, lenB2 ) ||
          min( lenA3, lenB3 ) * maximum_scale_factor_ < max( lenA3, lenB3 ) )
        continue;

      
      if( !minimal_solution_estimator_->estimate( subSet, rotMat, translation ) )
        continue; // no minimal solution found => linear dependent points !?
      else
        found = true;

      tempInliers->clear();

      for( unsigned cIdx = 0; cIdx < pointCorrespondences.size(); ++cIdx )
      {
        double temp, distanceSQR;
        const pair<Point3d,Point3d>& currentCorr = pointCorrespondences[ cIdx ];
        temp = row[0][0] * currentCorr.first.x +
               row[0][1] * currentCorr.first.y +
               row[0][2] * currentCorr.first.z +
               translation[0] - currentCorr.second.x;
        distanceSQR  = temp * temp;

        temp = row[1][0] * currentCorr.first.x +
               row[1][1] * currentCorr.first.y +
               row[1][2] * currentCorr.first.z +
               translation[1] - currentCorr.second.y;
        distanceSQR += temp * temp;

        temp = row[2][0] * currentCorr.first.x +
               row[2][1] * currentCorr.first.y +
               row[2][2] * currentCorr.first.z +
               translation[2] - currentCorr.second.z;
        distanceSQR += temp * temp;

        if( distanceSQR <= inlier_distance_threshold_SQR_ )
          tempInliers->push_back( cIdx );

      } // for

      #pragma omp critical
      if( tempInliers->size() > bestInliers->size() )
        swap( tempInliers, bestInliers );

    } while( drawCount < mininum_draws_ );

    delete tempInliers;
    } // omp-block

    vector<unsigned>* tempInliers = new vector<unsigned>();
    //cout << "maximum inliers: " << bestInliers->size() << " :: threshold: " << inlier_distance_threshold_SQR_ << endl;
    if( found ) // further refinement
    {
      vector<double> error_sqr( pointCorrespondences.size() );
      unsigned refineIt = 0;
      bool inlierChanged;
      double error_threshold_sqr = inlier_distance_threshold_SQR_;

      double* row[3];
      row[0] = rotMat.ptr<double>( 0 );
      row[1] = rotMat.ptr<double>( 1 );
      row[2] = rotMat.ptr<double>( 2 );
      do {
        vector<pair<Point3d,Point3d> > inliers;
        inliers.reserve( bestInliers->size() );
        for( vector<unsigned>::iterator it = bestInliers->begin(); it != bestInliers->end(); ++it )
          inliers.push_back( pointCorrespondences[*it] );

        // refine with all inliers
        minimal_solution_estimator_->estimate( inliers, rotMat, translation );
        tempInliers->clear();

        vector<double> error_sqr_inliers;
        for( unsigned cIdx = 0; cIdx < pointCorrespondences.size(); ++cIdx )
        {
          double temp, distanceSQR;
          const pair<Point3d,Point3d>& currentCorr = pointCorrespondences[ cIdx ];
          temp = row[0][0] * currentCorr.first.x +
                 row[0][1] * currentCorr.first.y +
                 row[0][2] * currentCorr.first.z +
                 translation[0] - currentCorr.second.x;
          distanceSQR  = temp * temp;

          temp = row[1][0] * currentCorr.first.x +
                 row[1][1] * currentCorr.first.y +
                 row[1][2] * currentCorr.first.z +
                 translation[1] - currentCorr.second.y;
          distanceSQR += temp * temp;

          temp = row[2][0] * currentCorr.first.x +
                 row[2][1] * currentCorr.first.y +
                 row[2][2] * currentCorr.first.z +
                 translation[2] - currentCorr.second.z;
          distanceSQR += temp * temp;

          if( distanceSQR <= error_threshold_sqr )
          {
            tempInliers->push_back( cIdx );
            error_sqr_inliers.push_back( distanceSQR );
          }
        }
        
        if( tempInliers->size() < 3 )
          return false;

        sort( error_sqr_inliers.begin(), error_sqr_inliers.end() );

        double median_error_sqr = error_sqr_inliers[ error_sqr_inliers.size() >> 1 ];
        double variance = 2.1981 * median_error_sqr;
        error_threshold_sqr = std::min( inlier_distance_threshold_SQR_, 9.0 * variance );

        swap( tempInliers , bestInliers );

        inlierChanged = false;
        if( bestInliers->size() != tempInliers->size() )
        {
          inlierChanged = true;
        }
        else
        {
          for( unsigned iIdx = 0; iIdx < tempInliers->size(); ++iIdx )
          {
            if( (*tempInliers)[iIdx] != (*bestInliers)[iIdx] )
            {
              inlierChanged = true;
              break;
            }
          }
        }
      } while( inlierChanged && ++refineIt < (unsigned) MINIMUM_DRAWS );
    }

    // copy the inliers
    inlier_indices_ = *bestInliers;

    #if __DEBUG_RANSAC__
    cout << "statistics:"<<endl;
    cout << "too small triangle sides: " << invalid_subset[0] << endl;
    cout << "invalid scale factor    : " << invalid_subset[1] << endl;
    cout << "invalid SVD solution    : " << invalid_subset[2] << endl;
    #endif
    
    delete tempInliers;
    delete bestInliers;
    return found;
  }

  RigidTransformationEstimator* RANSACRigidTransformationEstimator::clone() const
  {
    return new RANSACRigidTransformationEstimator( *this );
  }
} //namespace stereo_object_recognition

