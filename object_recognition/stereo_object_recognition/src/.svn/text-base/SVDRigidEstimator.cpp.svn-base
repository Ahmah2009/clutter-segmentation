#include "stereo_object_recognition/SVDRigidEstimator.h"
#include <iostream>

using namespace std;
using namespace cv;


#define USE_OPENCV_CPP 1
#define USE_OPENCV 0
#define USE_EIGEN 0

#if USE_EIGEN
    #include <Eigen/Core>
    #include <Eigen/SVD>
    using namespace Eigen;
#endif



namespace stereo_object_recognition
{

  SVDRigidEstimator::SVDRigidEstimator( double epsilon )
   : epsilon_( epsilon )
  {
  }

  SVDRigidEstimator::~SVDRigidEstimator( )
  {
  }

  bool SVDRigidEstimator::estimate( const vector<pair<Point3d,Point3d> >& point_correspondences, Mat& rotation_matrix, Vec3d& translation_vector ) const
  {
    // need at least 3 point correspondences!
    if( point_correspondences.size() < 3 )
      return false;

    // first find the centroids of both point sets
    double centroid1[3] = { 0.0, 0.0, 0.0 };
    double centroid2[3] = { 0.0, 0.0, 0.0 };
    for( vector<pair<Point3d,Point3d> >::const_iterator it = point_correspondences.begin(); it != point_correspondences.end(); ++it )
    {
      centroid1[0] += it->first.x;
      centroid1[1] += it->first.y;
      centroid1[2] += it->first.z;

      centroid2[0] += it->second.x;
      centroid2[1] += it->second.y;
      centroid2[2] += it->second.z;
    }
    double norm = 1.0 / (double)point_correspondences.size();
    centroid1[0] *= norm;
    centroid1[1] *= norm;
    centroid1[2] *= norm;

    centroid2[0] *= norm;
    centroid2[1] *= norm;
    centroid2[2] *= norm;

    double dataA[9] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    for( vector<pair<Point3d,Point3d> >::const_iterator it = point_correspondences.begin(); it != point_correspondences.end(); ++it )
    {
      dataA[0] += (it->first.x - centroid1[0]) * (it->second.x - centroid2[0]);
      dataA[1] += (it->first.x - centroid1[0]) * (it->second.y - centroid2[1]);
      dataA[2] += (it->first.x - centroid1[0]) * (it->second.z - centroid2[2]);

      dataA[3] += (it->first.y - centroid1[1]) * (it->second.x - centroid2[0]);
      dataA[4] += (it->first.y - centroid1[1]) * (it->second.y - centroid2[1]);
      dataA[5] += (it->first.y - centroid1[1]) * (it->second.z - centroid2[2]);

      dataA[6] += (it->first.z - centroid1[2]) * (it->second.x - centroid2[0]);
      dataA[7] += (it->first.z - centroid1[2]) * (it->second.y - centroid2[1]);
      dataA[8] += (it->first.z - centroid1[2]) * (it->second.z - centroid2[2]);
    }
    
    Mat A( 3, 3, CV_64FC1, (void*)dataA, sizeof(double) * 3 );

    Mat S( 3, 1, CV_64FC1 );
    Mat Ut( 3, 3, CV_64FC1 );
    Mat V( 3, 3, CV_64FC1 );

    svd( A, S, Ut, V );

    // colinear if two singular values are almost same
    if( fabs( S.at<double>(1,0) - S.at<double>(0,0) ) <= epsilon_ ||
        fabs( S.at<double>(2,0) - S.at<double>(1,0) ) <= epsilon_ )
    {
      return false;
    }

    rotation_matrix = V * Ut;
    double det = determinant( rotation_matrix );

    // if rot is a reflection => its determinant = -1
    if( det < 0.0 )
    {
      // is it coplanar?
      if( fabs( S.at<double>(2,0) ) <= epsilon_ )
      {
        // it is a reflection => make a rotation by inverting the column of V which corresponds to the smallest singular value!
        V.row(2) *= -1.0;
        rotation_matrix = V * Ut;
      }
      else
      {
        // its a reflection but its not coplanar!!!
        return false;
      }
    }

    // claculate translation vector
    Mat c1( 3, 1, CV_64FC1, centroid1, sizeof(double) );
    Mat c2( 3, 1, CV_64FC1, centroid2, sizeof(double) );

    Mat transMat = c2 - rotation_matrix * c1;
    translation_vector[0] = transMat.at<double>( 0, 0 );
    translation_vector[1] = transMat.at<double>( 1, 0 );
    translation_vector[2] = transMat.at<double>( 2, 0 );
    return true;
  }

  RigidTransformationEstimator* SVDRigidEstimator::clone() const
  {
    return new SVDRigidEstimator();
  }

  void SVDRigidEstimator::svd( Mat& mat, Mat& S, Mat& Ut, Mat& V )
  {
#if USE_OPENCV_CPP
    cv::SVD svd(mat);
    S = svd.w;
    Ut = svd.u.t();
    V  = svd.vt.t();
#elif USE_OPENCV
 // get always illegal instruction runtime error using cvSVD or even cv::SVD
    CvMat AA = (CvMat)mat;
    CvMat SS = (CvMat)S;
    CvMat UU = (CvMat)Ut;
    CvMat VV = (CvMat)V;

    cerr << __FILE__ << " @ " << __LINE__ << endl;
    cvSVD( &AA, &SS, &UU, &VV, CV_SVD_MODIFY_A | CV_SVD_U_T );
    cerr << __FILE__ << " @ " << __LINE__ << endl;
#else
    // !!!!!!!!!!!!!!!!! AA is the transposed of mat !!!!!!!!!!!!!!!!!!!!!
    Matrix3d AA( (double*)mat.data );

    Eigen::SVD<Matrix3d> svd(AA);
      
    for( int i = 0; i < 3; ++i )
    {
      for( int j = 0; j < 3; ++j )
      {
        Ut.at<double>( i, j ) = svd.matrixV()( j, i );
        V.at<double>( i, j ) = svd.matrixU()( i, j );
      }
      S.at<double>( i, 0 ) = svd.singularValues()( i, 0 );
    }
#endif
  }
} //namespace stereo_object_recognition

