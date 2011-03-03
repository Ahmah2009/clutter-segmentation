/* 
 * File:   ObjectDetectorNister.h
 * Author: gedikli
 *
 * Created on 12. Juni 2010, 23:01
 */

#ifndef STEREO_OBJECT_RECOGNITION_OBJECTDETECTORNISTER_H
#define	STEREO_OBJECT_RECOGNITION_OBJECTDETECTORNISTER_H

#include "vocabulary_tree/vocabulary_tree.h"
#include "vocabulary_tree/database.h"
#include "stereo_object_recognition/ObjectDetector.h"

namespace stereo_object_recognition
{
  template<typename TDescriptorType>
  class ObjectDetectorNister : public ObjectDetector
  {
    public:
      ObjectDetectorNister( const std::string& voc_tree_file_name,
                            const std::string& node_weight_file_name,
                            unsigned maximum_match_count = 10 );
      virtual ~ObjectDetectorNister( );
      inline virtual unsigned int addObject( const cv::Mat& feature_descriptors ) throw();
      inline virtual bool getBestMatchingObject( const cv::Mat& feature_descriptors, Match& bestMatch ) const throw();
      inline virtual std::vector<Match> getBestMatchingObjects( const cv::Mat& feature_descriptors ) const throw();
    protected:
      typedef Eigen::Matrix<TDescriptorType, 1, 176> Feature;
      unsigned maximum_match_count_;
      vt::VocabularyTree<Feature> vocabulary_tree_;
      vt::Database object_db_;
      void getDocument( const cv::Mat& feature_descriptors, vt::Document& document ) const;
  };

  template< typename TDescriptorType>
  ObjectDetectorNister<TDescriptorType>::ObjectDetectorNister( const std::string& voc_tree_file_name,
                                                               const std::string& node_weight_file_name,
                                                               unsigned maximum_match_count )
  : maximum_match_count_( maximum_match_count )
  , vocabulary_tree_( voc_tree_file_name )
  , object_db_( vocabulary_tree_.words() )
  {
    object_db_.loadWeights( node_weight_file_name );
  }

  template< typename TDescriptorType>
  ObjectDetectorNister<TDescriptorType>::~ObjectDetectorNister( )
  {
  }

  template< typename TDescriptorType>
  unsigned int ObjectDetectorNister<TDescriptorType>::addObject( const cv::Mat& feature_descriptors ) throw()
  {
    vt::Document document;
    getDocument( feature_descriptors, document );
    return (unsigned) object_db_.insert( document );
  }

  template< typename TDescriptorType>
  bool ObjectDetectorNister<TDescriptorType>::getBestMatchingObject( const cv::Mat& feature_descriptors, Match& bestMatch ) const throw()
  {
    vt::Document document;
    getDocument( feature_descriptors, document );
    vt::Matches matches;
    object_db_.find( document, 1, matches );

    if( matches.size() > 0 )
    {
      bestMatch.first = (unsigned)matches[0].id;
      bestMatch.second = matches[0].score;
      return true;
    }
    return false;
  }

  template< typename TDescriptorType>
  std::vector<ObjectDetector::Match> ObjectDetectorNister<TDescriptorType>::getBestMatchingObjects( const cv::Mat& feature_descriptors ) const throw()
  {
    vt::Document document;
    getDocument( feature_descriptors, document );
    vt::Matches matches;
    object_db_.find( document, maximum_match_count_, matches );

    std::vector<std::pair<unsigned int,float> > result( matches.size() );
    for( unsigned idx = 0; idx < matches.size(); ++idx )
    {
      result[idx].first = (unsigned)matches[idx].id;
      result[idx].second = matches[idx].score;
    }

    return result;
  }

  template< typename TDescriptorType>
  void ObjectDetectorNister<TDescriptorType>::getDocument( const cv::Mat& feature_descriptors, vt::Document& document ) const
  {
    Feature feature;
    for( int row_idx = 0; row_idx < feature_descriptors.rows; ++row_idx )
    {
      Feature feature;
      for( int col_idx = 0; col_idx < feature_descriptors.cols; ++col_idx )
      {
        feature( 1, col_idx ) = feature_descriptors.at<TDescriptorType>( row_idx, col_idx );
      }
      document.push_back( vocabulary_tree_.quantize( feature ) );
    }
  }
} // namespace stereo_object_recognition
#endif	/* STEREO_OBJECT_RECOGNITION_OBJECTDETECTORNISTER_H */

