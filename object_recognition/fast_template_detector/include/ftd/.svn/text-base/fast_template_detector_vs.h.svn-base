//////////////////////////////////////////////////////////////////////////////
//                                                                          //
// Copyright 2007 - 2010 Lehrstuhl fuer Informat XVI,                       //
// CAMP (Computer Aided Medical Procedures),                                //
// Technische Universitaet Muenchen, Germany.                               //
//                                                                          //
// All rights reserved.	This file is part of VISION.                        //
//                                                                          //
// VISION is free software; you can redistribute it and/or modify it        //
// under the terms of the GNU General Public License as published by        //
// the Free Software Foundation; either version 2 of the License, or        //
// (at your option) any later version.                                      //
//                                                                          //
// VISION is distributed in the hope that it will be useful, but            //
// WITHOUT ANY WARRANTY; without even the implied warranty of               //
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU         //
// General Public License for more details.                                 //
//                                                                          //
// You should have received a copy of the GNU General Public License        //
// along with VISION; if not, write to the Free Software Foundation,        //
// Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA        //
//                                                                          //
// Authors: Stefan Hinterstoisser, Stefan Holzer 2010                       //
// Version: 1.0                                                             //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////

#ifndef FTD_FAST_TEMPLATE_DETECTOR_VS_H_
#define FTD_FAST_TEMPLATE_DETECTOR_VS_H_ FTD_FAST_TEMPLATE_DETECTOR_VS_H_

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
 
  
//  static char bitsSetIn16bitVS [0x1u<<16];
//  static char bitsUnsetIn16bitVS [0x1u<<16];
  
  
       
  
  
  /**
   * \brief Fast template matching based on the work:
   *        "Dominant Orientation Templates for Real/Time Detection of
   *        Texture-Less Objects", CVPR 2010, San Francisco.
   */
  class FastTemplateDetectorVS
  {

  public: // functions

    /** \brief Creates a new object for learning and detecting templates. */
    FastTemplateDetectorVS (
      const int numOfHorizontalSamples, 
      const int numOfVerticalSamples, 
      const int samplingSize, 
      const int numOfCharsPerElement,
      const float minimumGradientMagnitude );
    /** \brief Destructs the object. */
    virtual ~FastTemplateDetectorVS ();
    
    
    /** \brief Returns the number of learned templates. */
    inline int 
      getNumOfTemplates () 
    { 
      return numOfLearnedTemplates_; 
    };
    /** \brief Returns the number of learned classes. */
    inline int 
      getNumOfClasses () 
    { 
      return numOfLearnedClasses_; 
    };
    /** \brief Returns the sampling size. */
    inline int 
      getSamplingSize () 
    { 
      return samplingSize_; 
    };
    /** \brief Returns the template width in pixels. */
    inline int 
      getTemplateWidth () 
    { 
      return samplingSize_*numOfHorizontalSamples_; 
    };
    /** \brief Returns the template height in pixels. */
    inline int 
      getTemplateHeight () 
    { 
      return samplingSize_*numOfVerticalSamples_; 
    };
    
    /** \brief Sets the number of pyramid levels which are used additionally to the original data. */
    void 
      setNumOfPyramidLevels (
        const int numOfPyramidLevels )
    {
      numOfPyramidLevels_ = numOfPyramidLevels;
    }
    
    /** \brief Returns the number of pyramid levels which are used additionally to the original data. */
    int
      getNumOfPyramidLevels (
        const int numOfPyramidLevels )
    {
      return numOfPyramidLevels_;
    }
    
    /** \brief Returns the list of templates. */
    List * getTemplateList() { return templatesStart_; };
    
    
    /** \brief Adds a new class and returns the corresponding class ID. */
    int 
      addNewClass ();
      
    /** \brief Adds a new template to the specified class. */
    void 
      addNewTemplate (
        unsigned char * templateData,
        const int classId,
        const int maxResponse = -1,
        CvRect roi = cvRect(-1, -1, -1, -1) );
        
    void
      addNewTemplate (
        unsigned char * templateData,
        unsigned char * mask,
        const int classId,
        const int maxResponse = -1 );
        
    void
      addContour (
        IplImage * templateImage );
        
    void
      drawContour (
        const int contourId,
        IplImage * image,
        const int startX,
        const int startY );
    void
      drawContour (
        const int contourId,
        IplImage * image,
        const int startX,
        const int startY,
        const float scale );
        
    int
      computeTemplateResponse (
        unsigned char * templateData,
        const int templateId );

        
    /** \brief Detects candidates for each class within the current image. */
    std::list< ::ftd::Candidate* > *
      process (
        unsigned char * data,
        const int threshold,
        const int horizontalSamples,
        const int verticalSamples,
        const bool getOnlyBestTemplatePerPosition = true );
        
    std::list< ::ftd::Candidate* > *
      process (
        unsigned char * data,
        unsigned char * mask,
        const int threshold,
        const int horizontalSamples,
        const int verticalSamples );
        
    std::list< ::ftd::Candidate* > *
      processMasked (
        unsigned char * data,
        unsigned char * mask,
        const int threshold,
        const int horizontalSamples,
        const int verticalSamples );
        
    /** \brief Detects candidates for each class within the current image. */
    std::list< ::ftd::Candidate* > *
      processWithPyramids (
        unsigned char * data,
        const int threshold,
        const int horizontalSamples,
        const int verticalSamples );
        
    void
      createTemplatePyramids ();
        
        
    /** \brief Clears the created clusters. */
    void 
      clearClusters ();
      
    /** \brief Creates clusters heuristically. */
    void 
      clusterHeuristically (
        const int max ); // TODO: max what???
        
        
    /** \brief Saves all the learned template to a file. */
    bool
      save (
        std::string name );
        
    /** \brief Loads templates from a file. */
    bool
      load (
        std::string name );
        
    /** \brief Adds the templates of the supplied detector to the template list of the current. */
    void
      combine (
        FastTemplateDetectorVS & detector );

    /** \brief Removes the specified template. */
    void
      removeTemplate (
        const int templateId );

  protected: // functions
  
    void 
      clearBitLists ();

          
    /** \brief Adds the bit list corresponding to a template to the specified class. */
    void 
      addBitList (
        unsigned char * bitList,
        const int classId,
        const int maxResponse = -1,
        const int occlusionAmount = 0,
        CvRect roi = cvRect(-1, -1, -1, -1) );

        
    /** \brief */
    void
      shiftInit (
        unsigned char * image,
        unsigned char * pix,
        const int width,
        const int height );
        
    /** \brief */
    void
      shiftRight (
        unsigned char * image,
        unsigned char * list,
        const int width,
        const int row,
        const int col );
        
    /** \brief */
    void
      shiftDown (
        unsigned char * image,
        unsigned char * pix,
        const int width,
        const int row,
        const int col );
        
    /** \brief */
    void 
      shiftCopy (
        unsigned char * oldData,
        unsigned char * newData );
        
        
    void
      shiftInit (
        unsigned char * image,
        const int imageWidth,
        const int imageHeight,
        unsigned char * subregion,
        const int subregionWidth,
        const int subregionHeight );
        
    void
      shiftRight (
        unsigned char * image,
        const int imageWidth,
        const int imageRow,
        const int imageCol,
        unsigned char * subregion,
        const int subregionWidth,
        const int subregionHeight,
        const int elementsInSubregion );
        
    void
      shiftDown (
        unsigned char * image,
        const int imageWidth,
        const int imageCol,
        const int imageRow,
        unsigned char * subregion,
        const int subregionWidth,
        const int subregionHeight,
        const int elementsInSubregion );
        
    void
      shiftCopy (
        unsigned char * oldData,
        unsigned char * newData,
        const int numOfElements );
        
    void
      copyData (
        unsigned char * image,
        const int imageWidth,
        const int imageHeight,
        const int imageCol,
        const int imageRow,
        unsigned char * subregion,
        const int subregionWidth,
        const int subregionHeight );

        
    /** \brief Computes a DOT candidate. */
    ::ftd::Candidate *
      computeCandidate (
        unsigned char * templateData, 
        const int threshold );
        
    void
      computeCandidates (
        unsigned char * templateData, 
        const int threshold,
        std::list< ::ftd::Candidate* > * candidateList,
        const int col,
        const int row );
        
    ::ftd::Candidate *
      computeCandidate (
        unsigned char * templateData, 
        const int threshold,
        const int occlusionAmount );
        
    ::ftd::Candidate *
      computeCandidateMasked (
        unsigned char * templateData, 
        const int threshold,
        const int occlusionAmount );
        
    ::ftd::Candidate *
      computeCandidate (
        unsigned char * templateData, 
        const int threshold,
        const int layerIndex,
        const int templateWidth,
        const int templateHeight );
        
    void
      computeCandidates (
        unsigned char * templateData, 
        const int threshold,
        const int layerIndex,
        const int templateWidth,
        const int templateHeight,
        std::list< ::ftd::Candidate * > * candidates,
        const int colIndex,
        const int rowIndex );
        
        
    /** \brief Transfers the current cluster data into the specified memory. */
    void
      transferBitList (
        unsigned char * memory );
        
        
    unsigned short
      clusterBitCount (
        unsigned char * pix1,
        unsigned char * pix2 );
        
    unsigned short
      bitSetsBitCount (
        unsigned char * pix1 );
        
    void
      computeOR (
        unsigned char * pix1,
        unsigned char * pix2,
        unsigned char * result );
        
        
    /** \brief Writes the learned templates to the specified stream. */
    std::ofstream & 
      write (
        std::ofstream & stream );
             
    /** \brief Reads templates from the specified stream. */
    std::ifstream &
      read (
        std::ifstream & stream );
    
    public:
        
      static std::ofstream & 
        write (
          std::ofstream & stream,
          IplImage * image );
          
      static IplImage *
        readImage (
          std::ifstream & stream );
        
        
	  int 
	    iteratedBitCount( 
	      unsigned short n )
	  {
		  int count = 0;

		  while (n)
		  {
			  count += n & 0x1u;    
			  n >>= 1;
		  }
		  
		  return count;
	  }

	  void 
	    computebitsSetIn16bitVS ()
	  {
		  for (unsigned int index = 0; index < (0x1u<<16); ++index)
		  {
			  bitsSetIn16bitVS_[index] = iteratedBitCount(index);
		  }	
		  
		  for (unsigned int index = 0; index < (0x1u<<16); ++index)
		  {
			  bitsUnsetIn16bitVS_[index] = 16-iteratedBitCount(index);
		  }
		  
		  return;
	  }
	  
	  
	  int 
	    iteratedBitCount16Bit( 
	      unsigned short n )
	  {
		  int count = 0;

		  if (((n & (3<<0)) >>0) == 3) ++count;
		  if (((n & (3<<2)) >>2) == 3) ++count;
		  if (((n & (3<<4)) >>4) == 3) ++count;
		  if (((n & (3<<6)) >>6) == 3) ++count;
		  if (((n & (3<<8)) >>8) == 3) ++count;
		  if (((n & (3<<10)) >>10) == 3) ++count;
		  if (((n & (3<<12)) >>12) == 3) ++count;
		  if (((n & (3<<14)) >>14) == 3) ++count;
		  if (((n & (3<<16)) >>16) == 3) ++count;
		  if (((n & (3<<18)) >>18) == 3) ++count;
		  if (((n & (3<<20)) >>20) == 3) ++count;
		  if (((n & (3<<22)) >>22) == 3) ++count;
		  if (((n & (3<<24)) >>24) == 3) ++count;
		  if (((n & (3<<26)) >>26) == 3) ++count;
		  if (((n & (3<<28)) >>28) == 3) ++count;
		  if (((n & (3<<30)) >>30) == 3) ++count;
		  
		  return count;
	  }
	  
	  
  unsigned short 
    get16bitEnergyBitCountLayerVS ( 
      __m128i * pix1, 
      __m128i * pix2, 
      const __m128i & zero,
      const int size );

  unsigned short 
    energyBitCountLayerVS (
      unsigned char * pix1, 
      unsigned char * pix2, 
      const __m128i & zero,
      const int size );
	  

  public: // functions
	  
	  void
	    printbitsSetIn16bitVS ()
	  {
	    for (unsigned int index = 0; index < (0x1u<<16); ++index)
	    {
	      std::cerr << index << ": " << static_cast<int> (bitsSetIn16bitVS_[index]) << std::endl;
	    } 
	  }
	  
	  void
	    changeTo16BitElements ()
	  {
		  for (unsigned int index = 0; index < (0x1u<<16); ++index)
		  {
			  bitsSetIn16bitVS_[index] = iteratedBitCount16Bit(index);
		  }	
		  
		  for (unsigned int index = 0; index < (0x1u<<16); ++index)
		  {
			  bitsUnsetIn16bitVS_[index] = 8-iteratedBitCount16Bit(index);
		  }
    }

                

  public: // data

    /** \brief The number of learned templates. */
    int numOfLearnedTemplates_;
    /** \brief The number of learned classes. */
    int numOfLearnedClasses_;
    /** \brief The number of elements per template (multiple of 16). */
    const int numOfElementsPerTemplate_;
    /** \brief The number of gradient bins used per template sample region. */
    const int numOfGradientBins_;
    /** \brief The minimum gradient magnitude that is considered. */
    const float minimumGradientMagnitude_;
    
    /** \brief Stores the templates. */
    List * templatesStart_;
    /** \brief Stores the clusters. */
    List * clusterStart_;
    
    std::vector<IplImage*> contours_;
    
    /** \brief Indicates whether templates are clustered or not. */
    bool areTemplatesClustered_;
    
    /** \brief The width of the currently processed image. */
    mutable int currentImageWidth_;
    /** \brief The height of the currently processed image. */
    mutable int currentImageHeight_;
    
    /** \brief */
    unsigned char * memory_;
    
    int numOfPyramidLevels_;
    
    int numOfHorizontalSamples_;
    int numOfVerticalSamples_;
    int samplingSize_;
    int numOfCharsPerElement_;
    
    char * bitsSetIn16bitVS_;
    char * bitsUnsetIn16bitVS_;
    
    
  };
   
}

#endif 


