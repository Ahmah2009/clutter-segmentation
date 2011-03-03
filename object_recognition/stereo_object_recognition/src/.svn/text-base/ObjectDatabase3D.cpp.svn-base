#include "stereo_object_recognition/ObjectDatabase3D.h"
#include "stereo_object_recognition/ObjectDetector.h"
#include <stdexcept>
#include <iostream>
#include <algorithm>
#include <boost/thread.hpp>
#include <boost/function.hpp>
#include <boost/timer.hpp>
#include <vector>

using namespace cv;
using namespace std;
using boost::shared_ptr;

#define USE_MULTI_THREADED true
#define USE_OPENCV_TRUNK false

namespace stereo_object_recognition
{

  ObjectDatabase3D::ObjectDatabase3D( shared_ptr<features2d::DescriptorMatcher> matcher,
                                      shared_ptr<GeometricValidator3D> validator, double max_descriptor_distance )
    : descriptor_matcher_( matcher )
    , geometric_validator_( validator )
    , max_descriptor_distance_( max_descriptor_distance )
    , object_detector_( (ObjectDetector*)0 )
  {
  }

  ObjectDatabase3D::ObjectDatabase3D( boost::shared_ptr<features2d::DescriptorMatcher> matcher,
                                      boost::shared_ptr<GeometricValidator3D> validator,
                                      boost::shared_ptr<ObjectDetector> object_detector, double max_descriptor_distance )
   : descriptor_matcher_( matcher )
   , geometric_validator_( validator )
   , max_descriptor_distance_( max_descriptor_distance )
   , object_detector_( object_detector )
  {
  }

  unsigned ObjectDatabase3D::addObject( const Mat& features, const vector<Point3d>& positionData )
  {
    if( features.rows != (int)positionData.size() )
      throw runtime_error("user data size does not match feature size");

    unsigned objectID = object_detector_->addObject( features );
    assert( objectID == objects_.size());
    objects_.push_back( pair<Mat, vector<Point3d> >( features, positionData ) );

    // return objectID = zero-based index of object
    return objectID;
  }

  vector<ObjectDatabase3D::ObjectMatch3D> ObjectDatabase3D::queryObject( const Mat& object_features,
                                                                         const vector<Point3d>& position_data ) const
  {
    // Iterate over all objects
    //   get Matching features
    //   if passes geometric check => candidate object
    vector<ObjectDatabase3D::ObjectMatch3D> result;
    ObjectMatch3D match3D;

    if( position_data.size() == 0 )
      return result;

    vector<ObjectDetector::Match> matches;
    if( object_detector_ != 0 ) {
      matches = object_detector_->getBestMatchingObjects( object_features );
    }

    boost::mutex result_mutex;
    pair<Mat, vector<Point3d> > queryObject(  object_features, position_data );

#if ( USE_MULTI_THREADED == true )
    unsigned cpu_count = boost::thread::hardware_concurrency();
    if( cpu_count == 0 ) // could not determine number of cpus!
        cpu_count = 1;

    unsigned start_index = 0;
    boost::thread_group job_package;

    unsigned jobs;
    if( object_detector_ != 0 )
      jobs = matches.size();
    else
      jobs = objects_.size();

    unsigned jobs_per_thread = jobs / cpu_count;
    int rest = jobs % cpu_count;

    for( unsigned thread_idx = 0; thread_idx < cpu_count; ++thread_idx )
    {
      unsigned end_index = start_index + jobs_per_thread;
      if ( rest-- > 0 )
          ++end_index;

      if( object_detector_ != 0 )
      {
        vector<ObjectDetector::Match>::const_iterator startIt = matches.begin() + start_index;
        vector<ObjectDetector::Match>::const_iterator endIt = matches.begin() + end_index;

        job_package.create_thread(boost::bind( &ObjectDatabase3D::queryObjectWorkerThreadLookUp, this, startIt, endIt,
                                               boost::ref(queryObject), boost::ref(result), boost::ref(result_mutex) ));
      }
      else
      {
        job_package.create_thread(boost::bind( &ObjectDatabase3D::queryObjectWorkerThread, this, start_index, end_index,
                                               boost::ref(queryObject), boost::ref(result), boost::ref(result_mutex) ));
      }
      start_index = end_index;
    }
    job_package.join_all();
#else

    if( object_detector_ == 0 )
    { // check all!

      queryObjectWorkerThread( 0, objects_.size(), boost::ref(queryObject), boost::ref(result), boost::ref(result_mutex) );
    }
    else // use matche from prefilter
    {
      queryObjectWorkerThreadLookUp( matches.begin(), matches.end(), boost::ref(queryObject),
                                     boost::ref(result), boost::ref(result_mutex) );
    }
#endif
    sort( result.begin(), result.end(), ObjectMatch3D() );
    return result;
  }

  unsigned ObjectDatabase3D::objectsCount() const
  {
    return objects_.size();
  }

  const std::pair<cv::Mat, std::vector<cv::Point3d> >& ObjectDatabase3D::getObjectFeatures( unsigned objectId ) const
  {
    // we do not check consistency of index... this has been done by the application!
    return objects_[objectId];
  }

  ostream& operator << ( ostream& os, const ObjectDatabase3D& objectDB )
  {
    os << (uint32_t) objectDB.objects_.size() << endl;
    for( vector<pair<Mat, vector<Point3d> > >::const_iterator objIt = objectDB.objects_.begin(); objIt != objectDB.objects_.end(); ++objIt )
    {
      os << (uint32_t) objIt->first.rows << " " << (uint32_t) objIt->first.cols << "  " << (uint32_t) objIt->first.type() << endl;

      for( int rowIdx = 0; rowIdx < objIt->first.rows; ++rowIdx )
      {
        for( int colIdx = 0; colIdx < objIt->first.cols; ++colIdx )
          os << objIt->first.at<float>( rowIdx, colIdx ) << " ";

        os << endl;
      }
/*            for( Mat_<double>::const_iterator matIt = objIt->first.begin<double>(); matIt != objIt->first.end<double>(); ++matIt ) {
            os << (*matIt) << " ";
        }
      os << endl;*/
      for( vector<Point3d>::const_iterator pointIt = objIt->second.begin(); pointIt != objIt->second.end(); ++pointIt)
        os << (pointIt->x) << " " << (pointIt->y) << " " << (pointIt->z) << " ";

      os << endl;
    }
    return os;
  }

  istream& operator >> ( istream& is, ObjectDatabase3D& objectDB )
  {
    uint32_t temp1, temp2, temp3;
    is >> temp1;
    cout << "loading object DB with " << temp1 << " objects..." << std::flush;
    objectDB.objects_.resize( temp1 );

    for( vector<pair<Mat, vector<Point3d> > >::iterator objIt = objectDB.objects_.begin(); objIt != objectDB.objects_.end(); ++objIt )
    {
      is >> temp1 >> temp2 >> temp3;
      //cout << "decriptor: " << temp1 << " x " << temp2 << " : " << temp3 << endl;
      objIt->first.create( temp1, temp2, temp3 );
      for( int rowIdx = 0; rowIdx < objIt->first.rows; ++rowIdx )
      {
        for( int colIdx = 0; colIdx < objIt->first.cols; ++colIdx )
          is >> objIt->first.at<float>( rowIdx, colIdx );
      }

      if( objectDB.object_detector_ != 0 )
      {
        objectDB.object_detector_->addObject( objIt->first );
      }
/*
        for( Mat_<double>::iterator matIt = objIt->first.begin<double>(); matIt != objIt->first.end<double>(); ++matIt ) {
            is >> (*matIt);
        }
*/
      //cout << "positions: " << temp1 << endl;
      objIt->second.resize( temp1 );
      for( vector<Point3d>::iterator pointIt = objIt->second.begin(); pointIt != objIt->second.end(); ++pointIt)
        is >> (pointIt->x) >> (pointIt->y) >> (pointIt->z);
    }

    // build the object_detector!
    cout << "done"<<endl;

    return is;
  }

  bool ObjectDatabase3D::matchObjects( const std::pair<cv::Mat, std::vector<cv::Point3d> >& object1,
                                       const std::pair<cv::Mat, std::vector<cv::Point3d> >& object2,
                                       ObjectMatch3D& match3D ) const
  {
#if (USE_OPENCV_TRUNK)
    vector<DMatch> forward_match;
    descriptor_matcher_->match( object1.first, object2.first, forward_match, Mat() );

    vector<DMatch> backward_match;
    descriptor_matcher_->match( object2.first, object1.first, backward_match, Mat() );
#else
    static boost::mutex mtx;
    mtx.lock();
      vector<DMatch> forward_match;
      descriptor_matcher_->clear();
      descriptor_matcher_->add( vector<Mat>(1, object2.first) );
      descriptor_matcher_->match( object1.first, forward_match );

      vector<DMatch> backward_match;
      descriptor_matcher_->clear();
      descriptor_matcher_->add( vector<Mat>(1, object1.first) );
      descriptor_matcher_->match( object2.first, backward_match );
    mtx.unlock();
#endif
    // form point correspondences!
    vector<pair<Point3d,Point3d> > correspondences; //( max( it->secondsize(), positionData.size() ) );
    match3D.descriptor_match_indices.clear();
    match3D.descriptor_match_indices.reserve( object1.second.size() + object2.second.size() );
    match3D.descriptor_match_distances.clear();
    match3D.descriptor_match_distances.reserve( object1.second.size() + object2.second.size() );

    // iterate over all features for forward match
    for( vector<DMatch>::const_iterator it = forward_match.begin(); it != forward_match.end(); ++it )
    {
      if( it->distance <= max_descriptor_distance_ )
      {
        correspondences.push_back( pair<Point3d,Point3d>( object1.second[it->queryIdx], object2.second[it->trainIdx]) );
        match3D.descriptor_match_indices.push_back( pair<unsigned,unsigned>( it->queryIdx, it->trainIdx ) );
        match3D.descriptor_match_distances.push_back( it->distance );
      }
    }

    // add reverse matches if not already found by forward match!
    for( vector<DMatch>::const_iterator it = backward_match.begin(); it != backward_match.end(); ++it )
    {
      if( forward_match[it->trainIdx].trainIdx != it->queryIdx && it->distance <= max_descriptor_distance_ )
      {
        correspondences.push_back( pair<Point3d,Point3d>( object1.second[it->trainIdx], object2.second[it->queryIdx]) );
        match3D.descriptor_match_indices.push_back( pair<unsigned,unsigned>( it->trainIdx, it->queryIdx ) );
        match3D.descriptor_match_distances.push_back( it->distance );
      }
    }

    // use geometric check
    vector<unsigned> inlierIndices;
    if( geometric_validator_->validate(correspondences, match3D.rotation_matrix, match3D.translation_vector, inlierIndices ) )
    {
      match3D.geometric_match_indices.clear();
      match3D.geometric_match_indices.reserve( inlierIndices.size() );

      //match3D.score = inlierIndices.size();
      match3D.score = 0.0;
      for( vector<unsigned>::iterator it = inlierIndices.begin(); it != inlierIndices.end(); ++it )
      {
        match3D.geometric_match_indices.push_back( *it );

        match3D.score += weightFunction( match3D.descriptor_match_distances[ *it ] );
      }
      return true;
    }
    return false;
  }

  void ObjectDatabase3D::queryObjectWorkerThread( unsigned start_index, unsigned end_index,
                                                  const std::pair<cv::Mat, std::vector<cv::Point3d> >& queryObject,
                                                  std::vector<ObjectMatch3D>& results, boost::mutex& result_mutex ) const
  {
    std::vector<ObjectMatch3D> result;
    ObjectMatch3D match3D;
    for( unsigned object_index = start_index; object_index < end_index; ++object_index )
    {
      match3D.object_ID = object_index;
      if( matchObjects( objects_[object_index], queryObject, match3D ) )
      {
        result.push_back( match3D );
      }
    }

    result_mutex.lock();
      results.insert( results.end(), result.begin(), result.end() );
    result_mutex.unlock();
  }

  void ObjectDatabase3D::queryObjectWorkerThreadLookUp( std::vector<ObjectDetector::Match>::const_iterator begin,
                                                        std::vector<ObjectDetector::Match>::const_iterator end,
                                                        const std::pair<cv::Mat, std::vector<cv::Point3d> >& queryObject,
                                                        std::vector<ObjectMatch3D>& results, boost::mutex& result_mutex ) const
  {
    std::vector<ObjectMatch3D> result;
    ObjectMatch3D match3D;
    for( std::vector<ObjectDetector::Match>::const_iterator mIt = begin; mIt != end; ++mIt )
    {
      unsigned object_index = mIt->first;
      match3D.object_ID = object_index;
      //match3D.score = mIt->second;
      if( matchObjects( objects_[object_index], queryObject, match3D ) )
      {
          result.push_back( match3D );
      }
    }

    result_mutex.lock();
      results.insert( results.end(), result.begin(), result.end() );
    result_mutex.unlock();
  }

  double ObjectDatabase3D::weightFunction( double distance ) const
  {
    return 1.0/(1.0 + pow( distance/ max_descriptor_distance_, 4 ) );
  }
} // namespace stereo_object_recognition
