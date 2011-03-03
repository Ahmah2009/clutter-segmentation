#ifndef STEREO_OBJECT_RECOGNITION_OBJECT_DATABASE3D_H
#define STEREO_OBJECT_RECOGNITION_OBJECT_DATABASE3D_H

#include <algorithm>
#include "stereo_object_recognition/GeometricValidator3D.h"
#include "stereo_object_recognition/ObjectDetector.h"
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
 #include <opencv2/features2d/features2d.hpp>

 namespace features2d = cv;

namespace stereo_object_recognition
{
  /**
   * \author Suat Gedikli
   * \date 28. April 2010
   * \brief Database for objects given by a set of descriptors with corresponding 3d-points.
   */
  class ObjectDatabase3D
  {
    public:
    /**
     * \author Suat Gedikli
     * \date 28. April 2010
     * \brief describes a match between a query object an the corresponding object in the DB. The match is always
     * directed from the object in the DB to the query object. e.g. the rigid transformation is from DB-object to
     * queryObject and the first index of a correspondences is the feature index of the DB-object.
     */
    typedef struct _ObjectMatch3D
    {
      _ObjectMatch3D()
        : score( 0.0 )
        , rotation_matrix( 3, 3, CV_64FC1 )
      {
      }
      // the corresponding DB-object id
      unsigned    object_ID;
      // a score-value that indicates the matching quality. higher is better
      float       score;
      // the rigid transformation is from DB-object to query object
      cv::Mat     rotation_matrix;
      cv::Vec3d   translation_vector;
      // first index is from database object. Second index from the query object!!!
      std::vector<std::pair<unsigned,unsigned> > descriptor_match_indices;
      // the corresponding distances for each matching descriptor
      std::vector<double> descriptor_match_distances;
      
      // ith entry gives us the index of a correspondence in descriptor_match_indices
      std::vector<unsigned> geometric_match_indices;

      bool operator()( const _ObjectMatch3D& obj1, const _ObjectMatch3D& obj2 )
      {
        return obj1.score > obj2.score ;
      }
    } ObjectMatch3D;

    public:
      ObjectDatabase3D( boost::shared_ptr<features2d::DescriptorMatcher> matcher,
                        boost::shared_ptr<GeometricValidator3D> validator, double max_inlier_distance = 0.05 );
      ObjectDatabase3D( boost::shared_ptr<features2d::DescriptorMatcher> matcher,
                        boost::shared_ptr<GeometricValidator3D> validator,
                        boost::shared_ptr<ObjectDetector> object_detector, double max_inlier_distance = 0.05 );

      /**
      * \author Suat Gedikli
      * \date 28. July 2010
      * \brief virtual desctuctor
      */
      virtual ~ObjectDatabase3D(){}

      /**
      * \author Suat Gedikli
      * \date 28. July 2010
      * \brief returns the number of objects stored in the database
      * \return number of objects stored in the database
      */
      virtual unsigned objectsCount() const;

      /**
      * \author Suat Gedikli
      * \date 28. July 2010
      * \brief returns the object features (descriptors + corresponding 3D points) for the object given by object-id
      * \param[in] object_Id the id of the object
      * \return featuers (descriptors + corresponding 3D points) of the queried object with given object id
      */
      virtual const std::pair<cv::Mat, std::vector<cv::Point3d> >& getObjectFeatures( unsigned object_Id ) const;

      /**
      * \author Suat Gedikli
      * \date 28. July 2010
      * \brief adds a new object given by a set of descriptors and corresponding 3D points.
      * \param[in] features set of descriptors
      * \param[in] position_data corresponding 3D points
      * \return object id of the newly added object
      */
      virtual unsigned addObject( const cv::Mat& features, const std::vector<cv::Point3d>& position_data );

      /**
      * \author Suat Gedikli
      * \date 28. July 2010
      * \brief queries a set of matching objects for a given set of descriptors with corresponding 3D points.
      * \param[in] features set of descriptors
      * \param[in] position_data corresponding 3D points
      * \return array of ObjectMatch3D objects for each matching object in the database
      */
      virtual std::vector<ObjectMatch3D> queryObject( const cv::Mat& objectFeatures,
                                                      const std::vector<cv::Point3d>& position_data ) const;


      friend std::ostream& operator << ( std::ostream& os, const ObjectDatabase3D& object_DB );
      friend std::istream& operator >> ( std::istream& is, ObjectDatabase3D& object_DB );
    protected:
      // the used descriptor matcher
      boost::shared_ptr<features2d::DescriptorMatcher> descriptor_matcher_;
      // geometric validator
      boost::shared_ptr<GeometricValidator3D> geometric_validator_;
      // database entries
      std::vector<std::pair<cv::Mat, std::vector<cv::Point3d> > > objects_;
      // maximum allowed descriptor distance for a valid descriptor correspondence
      double max_descriptor_distance_;
      // object detector for prefiltering the database
      boost::shared_ptr<ObjectDetector> object_detector_;
      // a weight function that is used to calculte the score of a match
      double weightFunction( double distance ) const;

      /**
      * \author Suat Gedikli
      * \date 28. July 2010
      * \brief matches two objects
      * \param[in] object1 the first object
      * \param[in] object2 the second object
      * \param[out] match corresponding ObjectMatch3D object. Valid only if method returns true
      * \return true if objects match, false otherwise
      */
      bool matchObjects( const std::pair<cv::Mat, std::vector<cv::Point3d> >& object1,
                         const std::pair<cv::Mat, std::vector<cv::Point3d> >& object2,
                         ObjectMatch3D& match ) const;

     /**
      * \author Suat Gedikli
      * \date 28. July 2010
      * \brief worker thread comparing objectes within a range: This one is currently used.
      */
      void queryObjectWorkerThread( unsigned start_index, unsigned end_index,
                                    const std::pair<cv::Mat, std::vector<cv::Point3d> >& queryObject,
                                    std::vector<ObjectMatch3D>& results, boost::mutex& result_mutex ) const;

    
     /**
      * \author Suat Gedikli
      * \date 30. July 2010
      * \brief worker thread comparing objectes within a range of the prefilter result
      */
      void queryObjectWorkerThreadLookUp( std::vector<ObjectDetector::Match>::const_iterator begin,
                                          std::vector<ObjectDetector::Match>::const_iterator end,
                                          const std::pair<cv::Mat, std::vector<cv::Point3d> >& queryObject,
                                          std::vector<ObjectMatch3D>& results, boost::mutex& result_mutex ) const;

  };
} //namespace stereo_object_recognition

#endif // STEREO_OBJECT_RECOGNITION_OBJECT_DATABASE3D_H