/* 
 * File:   Normal3DProvider.h
 * \author Suat Gedikli
 * \date 28. April 2010
 */

#ifndef STEREO_OBJECT_RECOGNITION_NORMAL3D_PROVIDER_H
#define	STEREO_OBJECT_RECOGNITION_NORMAL3D_PROVIDER_H
#include <cv.h>

namespace stereo_object_recognition
{
  class Normal3DProvider
  {
    public:
      virtual ~Normal3DProvider()
      {
      }

      /**
       * \author Suat Gedikli
       * \date 28. April 2010
       * \return the cloned object
       */
      virtual Normal3DProvider* clone() const = 0;

      /**
       * \author Suat Gedikli
       * \date 28. April 2010
       * \brief Provides the 3D normal in camera coordinate system of a image point
       * \param[in] imagePoint image point
       * \param[out] normal the according 3D coorinate of the image point in camera coordiante frame
       * \return false if no normal for the input pixel could be determined, true otherwise
       */
      virtual bool getNormal( const cv::Point2d& image_point, cv::Vec3d& normal ) const = 0;
  };
} //namespace stereo_object_recognition
#endif	/* STEREO_OBJECT_RECOGNITION_NORMAL3D_PROVIDER_H */

