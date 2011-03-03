//////////////////////////////////////////////////////////////////////////////
//
// Authors: Stefan Holzer 2010 (holzers@willowgarage.com)
// Version: 1.0 
//
//////////////////////////////////////////////////////////////////////////////

#ifndef UTIL_IMAGE_UTILS_H_
#define UTIL_IMAGE_UTILS_H_ UTIL_IMAGE_UTILS_H_

#include <opencv/cv.h>
#include <opencv/highgui.h>





/**
 * \namespace Namespace util for utility stuff.
 * \brief Namespace for utility stuff.
 */
namespace util
{

  /**
   * \brief Class containing utily functionality for images.
   */
  class ImageUtils
  {
  
  public: // functions
  
    ImageUtils () {};
    virtual ~ImageUtils () {};
    
    /**
     * \brief Creates an 32-bit floating point color image from the supplied image. 
     */
    static IplImage *
      createColorImage32F (
        IplImage * image )
    {
      IplImage * colorImage = NULL;
      
      if (image->depth == 32)
      {
        if (image->nChannels == 3)
        {
          colorImage = static_cast<IplImage*>(cvClone(image));
        }
        else if (image->nChannels == 1)
        {
          colorImage = cvCreateImage(cvGetSize(image), IPL_DEPTH_32F, 3);
          cvCvtColor(image, colorImage, CV_GRAY2BGR);
        }
      }
      else if (image->depth == 8)
      {
        if (image->nChannels == 3)
        {
          colorImage = cvCreateImage(cvGetSize(image), IPL_DEPTH_32F, 3);
          cvConvert(image, colorImage);
        }
        else if (image->nChannels == 1)
        {
          IplImage * tmpGrayImage = cvCreateImage(cvGetSize(image), IPL_DEPTH_32F, 1);
          cvConvert(image, tmpGrayImage);
          
          colorImage = cvCreateImage(cvGetSize(image), IPL_DEPTH_32F, 3);
          cvCvtColor(tmpGrayImage, colorImage, CV_GRAY2BGR);
          
          cvReleaseImage(&tmpGrayImage);
        }
        else
        {
          return NULL;
        }
      }    
      else
      {
        return NULL;
      }
      
      return colorImage;
    }
    
  
    /**
     * \brief Creates an 32-bit floating point gray value image from the supplied image. 
     */
    static IplImage *
      createGrayImage32F (
        IplImage * image )
    {
      IplImage * grayImage = NULL;
      
      if (image->depth == 32)
      {
        grayImage = cvCreateImage(cvGetSize(image), IPL_DEPTH_32F, 1);
        cvCvtColor(image, grayImage, CV_BGR2GRAY);
      }
      else if (image->depth == 8)
      {
        if (image->nChannels == 3)
        {
          IplImage * tmpGrayImage = cvCreateImage(cvGetSize(image), IPL_DEPTH_8U, 1);
          cvCvtColor(image, tmpGrayImage, CV_BGR2GRAY);
          
          grayImage = cvCreateImage(cvGetSize(image), IPL_DEPTH_32F, 1);
          cvConvert(tmpGrayImage, grayImage);
          
          cvReleaseImage(&tmpGrayImage);
        }
        else if (image->nChannels == 1)
        {
          grayImage = cvCreateImage(cvGetSize(image), IPL_DEPTH_32F, 1);
          cvConvert(image, grayImage);
        }
        else
        {
          return NULL;
        }
      }
      else
      {
        return NULL;
      }
      
      return grayImage;
    }
  
  };
  
}


#endif // UTIL_IMAGE_UTILS_H_
