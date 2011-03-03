//////////////////////////////////////////////////////////////////////////////
// 
// Authors: Stefan Holzer 2010 
// Version: 1.0 
//  
//////////////////////////////////////////////////////////////////////////////

#ifndef FTD_DISCRETIZERS_GRADIENT_DISCRETIZER_H_
#define FTD_DISCRETIZERS_GRADIENT_DISCRETIZER_H_ FTD_DISCRETIZERS_GRADIENT_DISCRETIZER_H_

#include <util/util.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <emmintrin.h>

#include <fstream>
#include <iostream>
#include <vector>
#include <list>




/**
 * \namespace Namespace ftd for fast template detection.
 * \brief Namespace for fast template detection.
 */
namespace ftd
{

  /**
   * \namespace Namespace for discretizers.
   * \brief Namespace for discretizers.
   */
  namespace discretizers
  {
 
    class GradientDiscretizer
    {
        
    public: // functions
      
      GradientDiscretizer ();
      virtual ~GradientDiscretizer ();  
      
      /**
       * Discretizes the gradients in the supplied image, combines the discretized gradients 
       * within one region and stores them in the supplied array.
       */
      static void
        discretize (
          IplImage * image,
          const int regionWidth,
          const int regionHeight,
          unsigned char * discretizedData,
          const int numOfGradients = 8 );
          
      /**
       * Discretizes the gradients in the supplied image, combines the discretized gradients 
       * within one region and stores them in the supplied vector.
       */
      static void
        discretize (
          IplImage * image,
          const int regionWidth,
          const int regionHeight,
          std::vector<unsigned char> & discretizedData,
          const int numOfGradients = 8 );
          
          
      /**
       * Discretizes the gradients in the supplied image, combines the discretized gradients 
       * within one region and stores them in the supplied vector.
       */
      static void
        discretize (
          IplImage * image,
          const int regionWidth,
          const int regionHeight,
          std::vector<unsigned char> & discretizedData,
          std::vector<float> & strength,
          const int numOfGradients = 8 );
          
      /**
       * Discretizes the gradients in the supplied image, combines the discretized gradients 
       * within one region and stores them in the supplied vector. Only gradients within the
       * area specified by the mask are considered.
       */
      static void
        discretize (
          IplImage * image,
          IplImage * mask,
          const int regionWidth,
          const int regionHeight,
          std::vector<unsigned char> & discretizedData,
          const int numOfGradients = 8 );
          
      /**
       * Discretizes the gradients in the supplied image, combines the discretized gradients 
       * within one region and stores them in the supplied vector. Only gradients within the
       * area specified by the mask are considered.
       */
      static void
        discretize (
          IplImage * image,
          IplImage * mask,
          const int regionWidth,
          const int regionHeight,
          std::vector<unsigned char> & discretizedData,
          std::vector<float> & strength,
          const int numOfGradients = 8 );
          
    };
  
  }
}

#endif // FTD_DISCRETIZERS_GRADIENT_DISCRETIZER_H_

