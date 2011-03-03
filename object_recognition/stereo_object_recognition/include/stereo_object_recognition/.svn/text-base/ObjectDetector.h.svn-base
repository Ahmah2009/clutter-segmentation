/* 
 * File:   ObjectDetector.h
 * Author: gedikli
 *
 * Created on 12. Juni 2010, 22:36
 */

#ifndef STEREO_OBJECT_RECOGNITION_OBJECT_DETECTOR_H
#define	STEREO_OBJECT_RECOGNITION_OBJECT_DETECTOR_H
#include <vector>
#include <opencv2/core/core.hpp>

namespace stereo_object_recognition
{
  /**
   * \author Suat Gedikli
   * \date 5.Aug 2010
   * \brief object detector base class. Provides the interface to detect object-ids from a given set of descriptors.
   */
  class ObjectDetector
  {
    public:
      /**
       * \author Suat Gedikli
       * \date 5.Aug 2010
       * \brief Match result, containing the object id and a score for this match
       */
      typedef std::pair<unsigned int,float> Match;

      /**
       * \author Suat Gedikli
       * \date 5.Aug 2010
       * \brief virtual desctructor
       */
      virtual ~ObjectDetector() {}

      /**
       * \author Suat Gedikli
       * \date 5.Aug 2010
       * \brief adds a set of descriptors for a new object.
       * \return the resulting object id.
       */
      virtual unsigned int addObject( const cv::Mat& feature_descriptors ) throw() = 0;

      /**
       * \author Suat Gedikli
       * \date 5.Aug 2010
       * \brief determines the best matching object
       * \return true if a match could be found, false otherwise.
       */
      virtual bool getBestMatchingObject( const cv::Mat& feature_descriptors, Match& bestMatch ) const throw() = 0;

      /**
       * \author Suat Gedikli
       * \date 5.Aug 2010
       * \brief determines the best matching objects
       * \return array of object matches.
       */
      virtual std::vector<Match> getBestMatchingObjects( const cv::Mat& feature_descriptors ) const throw() = 0;
  };
}
#endif	/* STEREO_OBJECT_RECOGNITION_OBJECT_DETECTOR_H */

