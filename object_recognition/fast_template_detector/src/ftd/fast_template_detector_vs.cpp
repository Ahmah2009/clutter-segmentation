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

#include <ftd/fast_template_detector_vs.h>


#define MAX_NUM_OF_DOT_TEMPLATES 10000

#define DOT_NO_CLUSTERING

// ============================================================================


unsigned short 
ftd::
FastTemplateDetectorVS::
get16bitEnergyBitCountLayerVS ( 
  __m128i * pix1, 
  __m128i * pix2, 
  const __m128i & zero,
  const int size )
{
  if (size == 1)
  {
    return bitsUnsetIn16bitVS_[_mm_movemask_epi8 (_mm_cmpeq_epi8 (_mm_and_si128 (*pix1, *pix2), zero))];
  }
  else
  {
    return get16bitEnergyBitCountLayerVS (pix1, pix2, zero, size-1)
		  + bitsUnsetIn16bitVS_[_mm_movemask_epi8 (_mm_cmpeq_epi8 (_mm_and_si128 (pix1[size-1], pix2[size-1]), zero))];
  }
}


// ============================================================================


unsigned short 
ftd::
FastTemplateDetectorVS::
energyBitCountLayerVS (
  unsigned char * pix1, 
  unsigned char * pix2, 
  const __m128i & zero,
  const int size )
{
  return get16bitEnergyBitCountLayerVS (reinterpret_cast<__m128i*> (pix1), reinterpret_cast<__m128i*> (pix2), zero, size);
}


// ============================================================================


ftd::
FastTemplateDetectorVS::
FastTemplateDetectorVS (
  const int numOfHorizontalSamples, 
  const int numOfVerticalSamples, 
  const int samplingSize, 
  const int numOfCharsPerElement,
  const float minimumGradientMagnitude )
  : numOfLearnedTemplates_ (0),
    numOfLearnedClasses_ (0),
    numOfElementsPerTemplate_ (std::max (16, ((numOfHorizontalSamples*numOfVerticalSamples)/16+1)*16)),
    numOfGradientBins_ (8),
    minimumGradientMagnitude_ (minimumGradientMagnitude),
    templatesStart_ (NULL),
    clusterStart_ (NULL),
    areTemplatesClustered_ (false),
    numOfPyramidLevels_ (2),
    numOfHorizontalSamples_ (numOfHorizontalSamples),
    numOfVerticalSamples_ (numOfVerticalSamples),
    samplingSize_ (samplingSize),
    numOfCharsPerElement_ (numOfCharsPerElement)
{
  //memory_ = new unsigned char[numOfCharsPerElement_*numOfElementsPerTemplate_*MAX_NUM_OF_DOT_TEMPLATES];
  memory_ = reinterpret_cast<unsigned char*>(_mm_malloc(sizeof(unsigned char)*numOfCharsPerElement_*numOfElementsPerTemplate_*MAX_NUM_OF_DOT_TEMPLATES, 16));
  memset (memory_, 0, numOfCharsPerElement_*numOfElementsPerTemplate_*MAX_NUM_OF_DOT_TEMPLATES);

  bitsSetIn16bitVS_ = new char[0x1u<<16];
  bitsUnsetIn16bitVS_ = new char[0x1u<<16];  

  if (numOfCharsPerElement_ == 1)
  {
    computebitsSetIn16bitVS();
    
    //std::cerr << "numOfCharsPerElement_ == 1" << std::endl;
  }
  else if (numOfCharsPerElement_ == 2)
  {
    changeTo16BitElements();
    
    //std::cerr << "numOfCharsPerElement_ == 2" << std::endl;
  }
  else
  {
    std::cerr << "ERROR: unsupported numOfCharsPerElement" << std::endl;
  }
  
  //printbitsSetIn16bitVS ();
}


// ============================================================================


ftd::
FastTemplateDetectorVS::
~FastTemplateDetectorVS ()
{
  //delete[] memory_;
  _mm_free(memory_);
  
  delete[] bitsSetIn16bitVS_;
  delete[] bitsUnsetIn16bitVS_;
}


// ============================================================================


int
ftd::
FastTemplateDetectorVS::
addNewClass ()
{
  return numOfLearnedClasses_++;
}


// ============================================================================


void
ftd::
FastTemplateDetectorVS::
addNewTemplate (
  unsigned char * templateData,
  const int classId,
  const int maxResponse,
  CvRect roi )
{
  this->addBitList (templateData, classId, maxResponse, 0, roi);  
}


// ============================================================================


void
ftd::
FastTemplateDetectorVS::
addNewTemplate (
  unsigned char * templateData,
  unsigned char * mask,
  const int classId,
  const int maxResponse )
{
  // compute amount of occlusion
  int occlusionAmount = 0;

  //unsigned char * maskedData = new unsigned char[numOfCharsPerElement_*numOfElementsPerTemplate_];
  unsigned char * maskedData = reinterpret_cast<unsigned char*>(_mm_malloc(sizeof(unsigned char)*numOfCharsPerElement_*numOfElementsPerTemplate_, 16));
  memcpy (maskedData, templateData, numOfCharsPerElement_*numOfElementsPerTemplate_);

  for (int rowIndex = 0; rowIndex < numOfVerticalSamples_; ++rowIndex)
  {
    for (int colIndex = 0; colIndex < numOfHorizontalSamples_; ++colIndex)
    {
      if (mask[rowIndex*numOfHorizontalSamples_+colIndex] == 0)
      {
        ++occlusionAmount;
        
        for (int charIndex = 0; charIndex < numOfCharsPerElement_; ++charIndex)
        {
          maskedData[rowIndex*numOfCharsPerElement_*numOfHorizontalSamples_+numOfCharsPerElement_*colIndex+charIndex] = 0;
        }
      }
    }
  }
  
  this->addBitList (templateData, classId, maxResponse, occlusionAmount);  
  
  //delete[] maskedData;
  _mm_free(maskedData);
}


// ============================================================================


std::list< ::ftd::Candidate* > *
ftd::
FastTemplateDetectorVS::
process (
  unsigned char * data,
  const int threshold,
  const int horizontalSamples,
  const int verticalSamples,
  const bool getOnlyBestTemplatePerPosition )
{
  using ::ftd::Candidate;
  
  std::list<Candidate*> * candidateList = new std::list<Candidate*>[numOfLearnedClasses_];
  
  //std::cerr << "numOfCharsPerElement_: " << numOfCharsPerElement_ << std::endl;
  
  //unsigned char * pixelRow = new unsigned char[numOfCharsPerElement_*numOfElementsPerTemplate_];
  //unsigned char * pixelCol = new unsigned char[numOfCharsPerElement_*numOfElementsPerTemplate_];

  unsigned char * pixelRow = reinterpret_cast<unsigned char*>(_mm_malloc(sizeof(unsigned char)*numOfCharsPerElement_*numOfElementsPerTemplate_, 16));
  unsigned char * pixelCol = reinterpret_cast<unsigned char*>(_mm_malloc(sizeof(unsigned char)*numOfCharsPerElement_*numOfElementsPerTemplate_, 16));
  
  memset (pixelRow, 0, numOfCharsPerElement_*numOfElementsPerTemplate_);
  memset (pixelCol, 0, numOfCharsPerElement_*numOfElementsPerTemplate_);
  
  this->shiftInit (data, pixelRow, horizontalSamples, verticalSamples);
  this->shiftCopy (pixelRow, pixelCol);
   
  for (int rowIndex = 0; rowIndex < verticalSamples-numOfVerticalSamples_; ++rowIndex)
  {
    for (int colIndex = 0; colIndex < horizontalSamples-numOfHorizontalSamples_; ++colIndex)
    {
      if (getOnlyBestTemplatePerPosition)
      {
        Candidate * candidate = this->computeCandidate (pixelCol, threshold);
        
        if (candidate->getIndex () != 0)
        {
          candidate->setCol (colIndex*samplingSize_);
          candidate->setRow (rowIndex*samplingSize_);
          
          candidateList[candidate->getClassIndex ()].push_back (candidate);
        }
        else
        {
          delete candidate;
        }
      }
      else
      {
        this->computeCandidates (pixelCol, threshold, candidateList, colIndex*samplingSize_, rowIndex*samplingSize_);
      }
      
      this->shiftRight (data, pixelCol, horizontalSamples, rowIndex, colIndex);
    }
    
    this->shiftDown (data, pixelRow, horizontalSamples, rowIndex, 0);
    this->shiftCopy (pixelRow, pixelCol);
  }
  
  //delete[] pixelCol;
  //delete[] pixelRow;

  _mm_free(pixelCol);
  _mm_free(pixelRow);

  return candidateList;
}


// ============================================================================


std::list< ::ftd::Candidate* > *
ftd::
FastTemplateDetectorVS::
process (
  unsigned char * data,
  unsigned char * mask,
  const int threshold,
  const int horizontalSamples,
  const int verticalSamples )
{
  using ::ftd::Candidate;
  
  
  // compute amounts of occlusion
  unsigned char * maskedData = new unsigned char[numOfCharsPerElement_*horizontalSamples*verticalSamples];
  memcpy (maskedData, data, numOfCharsPerElement_*horizontalSamples*verticalSamples*sizeof (unsigned char));
  
  int * occlusionIntegralImage = new int[horizontalSamples*verticalSamples];
  memset (occlusionIntegralImage, 0, horizontalSamples*verticalSamples*sizeof (int));
  

  for (int rowIndex = 1; rowIndex < verticalSamples; ++rowIndex)
  {
    for (int colIndex = 1; colIndex < horizontalSamples; ++colIndex)
    {
      if (mask[rowIndex*horizontalSamples+colIndex] == 0)
      {
        for (int charIndex = 0; charIndex < numOfCharsPerElement_; ++charIndex)
        {
          maskedData[rowIndex*numOfCharsPerElement_*horizontalSamples+numOfCharsPerElement_*colIndex+charIndex] = 0;
        }
        
        occlusionIntegralImage[rowIndex*horizontalSamples+colIndex] = 1;
      }
      
      if (rowIndex > 0)
      {
        occlusionIntegralImage[rowIndex*horizontalSamples+colIndex] += occlusionIntegralImage[(rowIndex-1)*horizontalSamples+colIndex];
      }
      if (colIndex > 0)
      {
        occlusionIntegralImage[rowIndex*horizontalSamples+colIndex] += occlusionIntegralImage[rowIndex*horizontalSamples+(colIndex-1)];
      }
      if (rowIndex > 0 && colIndex > 0)
      {
        occlusionIntegralImage[rowIndex*horizontalSamples+colIndex] -= occlusionIntegralImage[(rowIndex-1)*horizontalSamples+(colIndex-1)];
      }
    }
  }  
  
  
  std::list<Candidate*> * candidateList = new std::list<Candidate*>[numOfLearnedClasses_];
  
  //std::cerr << "numOfCharsPerElement_: " << numOfCharsPerElement_ << std::endl;
  
  //std::cerr << "test0.1" << std::endl;
  //unsigned char * pixelRow = new unsigned char[numOfCharsPerElement_*numOfElementsPerTemplate_];
  //unsigned char * pixelCol = new unsigned char[numOfCharsPerElement_*numOfElementsPerTemplate_];

  unsigned char * pixelRow = reinterpret_cast<unsigned char*>(_mm_malloc(sizeof(unsigned char)*numOfCharsPerElement_*numOfElementsPerTemplate_, 16));
  unsigned char * pixelCol = reinterpret_cast<unsigned char*>(_mm_malloc(sizeof(unsigned char)*numOfCharsPerElement_*numOfElementsPerTemplate_, 16));

  
  //std::cerr << "test0.2" << std::endl;
  memset (pixelRow, 0, numOfCharsPerElement_*numOfElementsPerTemplate_);
  memset (pixelCol, 0, numOfCharsPerElement_*numOfElementsPerTemplate_);
  
  IplImage * occlusionImage = cvCreateImage(cvSize(horizontalSamples, verticalSamples), IPL_DEPTH_32F, 1);
  
  //std::cerr << "test1" << std::endl;
  this->shiftInit (data, pixelRow, horizontalSamples, verticalSamples);
  this->shiftCopy (pixelRow, pixelCol);
   
  //std::cerr << "test2" << std::endl;
  for (int rowIndex = 0; rowIndex < verticalSamples-numOfVerticalSamples_; ++rowIndex)
  {
    for (int colIndex = 0; colIndex < horizontalSamples-numOfHorizontalSamples_; ++colIndex)
    {
      // compute amount of occlusion
      int occlusionAmount = occlusionIntegralImage[rowIndex*horizontalSamples+colIndex]
        + occlusionIntegralImage[(rowIndex+numOfVerticalSamples_)*horizontalSamples+colIndex+numOfHorizontalSamples_]
        - occlusionIntegralImage[(rowIndex+numOfVerticalSamples_)*horizontalSamples+colIndex]
        - occlusionIntegralImage[rowIndex*horizontalSamples+colIndex+numOfHorizontalSamples_];

      CV_IMAGE_ELEM (occlusionImage, float, rowIndex, colIndex) = occlusionAmount;

      Candidate * candidate = this->computeCandidate (pixelCol, threshold, occlusionAmount);
      
      if (candidate->getIndex () != 0)
      {
        candidate->setCol (colIndex*samplingSize_);
        candidate->setRow (rowIndex*samplingSize_);
        
        candidateList[candidate->getClassIndex ()].push_back (candidate);
      }
      else
      {
        delete candidate;
      }
      
      //std::cerr << "test3" << std::endl;
      this->shiftRight (data, pixelCol, horizontalSamples, rowIndex, colIndex);
    }
    
    //std::cerr << "test4" << std::endl;
    this->shiftDown (data, pixelRow, horizontalSamples, rowIndex, 0);
    //std::cerr << "test5" << std::endl;
    this->shiftCopy (pixelRow, pixelCol);
  }
  //std::cerr << "test6" << std::endl;
  
  //delete[] pixelCol;
  //delete[] pixelRow;

  _mm_free(pixelCol);
  _mm_free(pixelRow);

  cvScale (occlusionImage, occlusionImage, 1.0/255.0);
  cvShowImage ("occlusionImage", occlusionImage);
  cvReleaseImage (&occlusionImage);

  //std::cerr << "test8" << std::endl;
  return candidateList;
}


// ============================================================================


std::list< ::ftd::Candidate* > *
ftd::
FastTemplateDetectorVS::
processMasked (
  unsigned char * data,
  unsigned char * mask,
  const int threshold,
  const int horizontalSamples,
  const int verticalSamples )
{
  using ::ftd::Candidate;
  
  
  // compute amounts of occlusion
  unsigned char * maskedData = new unsigned char[numOfCharsPerElement_*horizontalSamples*verticalSamples];
  memcpy (maskedData, data, numOfCharsPerElement_*horizontalSamples*verticalSamples*sizeof (unsigned char));
  
  int * occlusionIntegralImage = new int[horizontalSamples*verticalSamples];
  memset (occlusionIntegralImage, 0, horizontalSamples*verticalSamples*sizeof (int));
  

  for (int rowIndex = 1; rowIndex < verticalSamples; ++rowIndex)
  {
    for (int colIndex = 1; colIndex < horizontalSamples; ++colIndex)
    {
      if (mask[rowIndex*horizontalSamples+colIndex] == 0)
      {
        for (int charIndex = 0; charIndex < numOfCharsPerElement_; ++charIndex)
        {
          maskedData[rowIndex*numOfCharsPerElement_*horizontalSamples+numOfCharsPerElement_*colIndex+charIndex] = 0;
        }
        
        occlusionIntegralImage[rowIndex*horizontalSamples+colIndex] = 1;
      }
      
      if (rowIndex > 0)
      {
        occlusionIntegralImage[rowIndex*horizontalSamples+colIndex] += occlusionIntegralImage[(rowIndex-1)*horizontalSamples+colIndex];
      }
      if (colIndex > 0)
      {
        occlusionIntegralImage[rowIndex*horizontalSamples+colIndex] += occlusionIntegralImage[rowIndex*horizontalSamples+(colIndex-1)];
      }
      if (rowIndex > 0 && colIndex > 0)
      {
        occlusionIntegralImage[rowIndex*horizontalSamples+colIndex] -= occlusionIntegralImage[(rowIndex-1)*horizontalSamples+(colIndex-1)];
      }
    }
  }  
  
  
  std::list<Candidate*> * candidateList = new std::list<Candidate*>[numOfLearnedClasses_];
  
  //std::cerr << "numOfCharsPerElement_: " << numOfCharsPerElement_ << std::endl;
  
  //std::cerr << "test0.1" << std::endl;
  //unsigned char * pixelRow = new unsigned char[numOfCharsPerElement_*numOfElementsPerTemplate_];
  //unsigned char * pixelCol = new unsigned char[numOfCharsPerElement_*numOfElementsPerTemplate_];

  unsigned char * pixelRow = reinterpret_cast<unsigned char*>(_mm_malloc(sizeof(unsigned char)*numOfCharsPerElement_*numOfElementsPerTemplate_, 16));
  unsigned char * pixelCol = reinterpret_cast<unsigned char*>(_mm_malloc(sizeof(unsigned char)*numOfCharsPerElement_*numOfElementsPerTemplate_, 16));

  //std::cerr << "test0.2" << std::endl;
  memset (pixelRow, 0, numOfCharsPerElement_*numOfElementsPerTemplate_);
  memset (pixelCol, 0, numOfCharsPerElement_*numOfElementsPerTemplate_);
  
  IplImage * occlusionImage = cvCreateImage(cvSize(horizontalSamples, verticalSamples), IPL_DEPTH_32F, 1);
  
  //std::cerr << "test1" << std::endl;
  this->shiftInit (maskedData, pixelRow, horizontalSamples, verticalSamples);
  this->shiftCopy (pixelRow, pixelCol);
   
  //std::cerr << "test2" << std::endl;
  for (int rowIndex = 0; rowIndex < verticalSamples-numOfVerticalSamples_; ++rowIndex)
  {
    for (int colIndex = 0; colIndex < horizontalSamples-numOfHorizontalSamples_; ++colIndex)
    {
      // compute amount of occlusion
      int occlusionAmount = occlusionIntegralImage[rowIndex*horizontalSamples+colIndex]
        + occlusionIntegralImage[(rowIndex+numOfVerticalSamples_)*horizontalSamples+colIndex+numOfHorizontalSamples_]
        - occlusionIntegralImage[(rowIndex+numOfVerticalSamples_)*horizontalSamples+colIndex]
        - occlusionIntegralImage[rowIndex*horizontalSamples+colIndex+numOfHorizontalSamples_];

      CV_IMAGE_ELEM (occlusionImage, float, rowIndex, colIndex) = occlusionAmount;

      Candidate * candidate = this->computeCandidateMasked (pixelCol, threshold, occlusionAmount);
      
      if (candidate->getIndex () != 0)
      {
        candidate->setCol (colIndex*samplingSize_);
        candidate->setRow (rowIndex*samplingSize_);
        
        candidateList[candidate->getClassIndex ()].push_back (candidate);
      }
      else
      {
        delete candidate;
      }
      
      //std::cerr << "test3" << std::endl;
      this->shiftRight (maskedData, pixelCol, horizontalSamples, rowIndex, colIndex);
    }
    
    //std::cerr << "test4" << std::endl;
    this->shiftDown (maskedData, pixelRow, horizontalSamples, rowIndex, 0);
    //std::cerr << "test5" << std::endl;
    this->shiftCopy (pixelRow, pixelCol);
  }
  //std::cerr << "test6" << std::endl;
  
  //delete[] pixelCol;
  //delete[] pixelRow;

  _mm_free(pixelCol);
  _mm_free(pixelRow);


  cvScale (occlusionImage, occlusionImage, 1.0/255.0);
  cvShowImage ("occlusionImage", occlusionImage);
  cvReleaseImage (&occlusionImage);

  //std::cerr << "test8" << std::endl;
  return candidateList;
}


// ============================================================================


void
ftd::
FastTemplateDetectorVS::
clearClusters ()
{
  using ::ftd::List;
  
  List * currentPointer = clusterStart_;
  
  while (currentPointer != NULL)
  {
    List * deletePointer = currentPointer;
    currentPointer = currentPointer->getNext ();
    delete[] deletePointer->getBitList ();
    delete deletePointer;
  }
  
  clusterStart_ = NULL;
}


// ============================================================================


void
ftd::
FastTemplateDetectorVS::
clusterHeuristically (
  const int max )
{
  using ::ftd::List;
  
  if (numOfLearnedTemplates_ == 0)
  {
    return;
  }
  
  this->clearClusters ();
  
  clusterStart_ = NULL;
  List * currentElement = NULL;
  currentElement = templatesStart_;
  
  while (currentElement != NULL)
  {
    currentElement->setClusterIndex (0);
    currentElement->setFLW (NULL);
    currentElement = currentElement->getNext ();
  }
  
  currentElement = clusterStart_;
  
  int clusterNumber = 1;
  bool flag = true;
  
  
  while (flag)
  {
    int tmpMax = -1;
    flag = false;
    
    List * currentElement0 = templatesStart_;
    List * currentElement1 = templatesStart_;
    
    while (currentElement0 != NULL)
    {
      if (currentElement0->getClusterIndex () == 0)
      {
        unsigned short value = bitSetsBitCount (currentElement0->getBitList ());
        
        if (value > tmpMax)
        {
          tmpMax = value;
          currentElement1 = currentElement0;
        }
        flag = true;
      }
      
      currentElement0 = currentElement0->getNext ();
    }
    
    if (currentElement1->getClusterIndex () == 0)
    {
      bool end = false;
      int counter = 1;
      
      List * cluster = new List();
      
      currentElement1->setClusterIndex (clusterNumber);
      
      cluster->setClusterIndex (clusterNumber);
      cluster->setIndex (clusterNumber);
      cluster->setBitList (new unsigned char[numOfCharsPerElement_*numOfElementsPerTemplate_]);
      cluster->setFLW (currentElement1);
      cluster->setNext (NULL);
      
      memcpy (cluster->getBitList (), currentElement1->getBitList (), numOfCharsPerElement_*numOfElementsPerTemplate_);
      //unsigned char * clusterBitList = cluster->getBitList ();
      //unsigned char * currentElement1BitList = currentElement1->getBitList ();
      //for (int index = 0; index < numOfCharsPerElement_*numOfElementsPerTemplate_; ++index)
      //{
      //  clusterBitList[index] = currentElement1BitList[index];
      //}
      
      List * ind1 = currentElement1;
      
      while ( counter < max
           && end == false )
      {
        List * currentElement2 = templatesStart_;
        List * ind2 = NULL;
        
        int min = numOfElementsPerTemplate_*numOfGradientBins_; // numOfCharsPerElement_* ???
        
        end = true;
        
        while (currentElement2 != NULL)
        {
          if (currentElement2->getClusterIndex () == 0)
          {
            unsigned short value = clusterBitCount (cluster->getBitList (), currentElement2->getBitList ());
            
            if (value < min)
            {
              min = value;
              end = false;
              ind2 = currentElement2;
            }
          }
          
          currentElement2 = currentElement2->getNext ();
        }
                
        if (ind2 != NULL)
        {
          unsigned char * ind2BitList = ind2->getBitList ();
          unsigned char * clusterBitList = cluster->getBitList ();
          
          for (int index = 0; index < numOfCharsPerElement_*numOfElementsPerTemplate_; ++index)
          {
            clusterBitList[index] = clusterBitList[index] | ind2BitList[index];
          }
          
          ind2->setClusterIndex (clusterNumber);
          ind1->setFLW (ind2);
          ind1 = ind2;
          
          ++counter;
        }
      }
      
      if (clusterStart_ == NULL)
      {
        clusterStart_ = cluster;
      }
      else
      {
        List * currentCluster = clusterStart_;
        
        while (currentCluster->getNext () != NULL)
        {
          currentCluster = currentCluster->getNext ();
        }
        
        currentCluster->setNext (cluster);
      }
      
      ++clusterNumber;
    }
  }
    
  areTemplatesClustered_ = true;
  
  this->transferBitList(memory_);
  
  return;
}


// ============================================================================


bool
ftd::
FastTemplateDetectorVS::
save (
  std::string name )
{
  std::ofstream fileStream (
    name.c_str (),
    std::ofstream::out | std::ofstream::binary );
    
  if (fileStream.fail ())
  {
    std::cerr << "FastTemplateDetectorVS: could not open file for writing" << std::endl;
    return false;
  }
  
  this->write (fileStream);
  
  fileStream.close ();
  
  return true;
}

        
// ============================================================================


bool
ftd::
FastTemplateDetectorVS::
load (
  std::string name )
{
  std::ifstream fileStream (
    name.c_str (),
    std::ifstream::in | std::ifstream::binary );
    
  if (fileStream.fail ())
  {
    std::cerr << "FastTemplateDetectorVS: could not open file for reading" << std::endl;
    return false;
  }
  
  this->read (fileStream);
  
  fileStream.close ();
  
  return true;
}


// ============================================================================


void
ftd::
FastTemplateDetectorVS::
clearBitLists ()
{
  using ::ftd::List;
  
  List * currentTemplate = templatesStart_;
  
  while (currentTemplate != NULL)
  {
    List * deleteTemplate = currentTemplate;
    currentTemplate = currentTemplate->getNext ();
    //delete[] (deleteTemplate->getBitList ());
    _mm_free(deleteTemplate->getBitList ());
    delete deleteTemplate;
  }
  
  templatesStart_ = NULL;
  
  numOfLearnedTemplates_ = 0;
  numOfLearnedClasses_ = 0;
}


// ============================================================================


void
ftd::
FastTemplateDetectorVS::
addBitList (
  unsigned char * bitList,
  const int classId,
  const int maxResponse,
  const int occlusionAmount,
  CvRect roi )
{
  areTemplatesClustered_ = false;
  
  if (templatesStart_ == NULL)
  {
    templatesStart_ = new List ();
    templatesStart_->setIndex (1);
    templatesStart_->setClusterIndex (0);
    templatesStart_->setClassIndex (classId);
    templatesStart_->setNext (NULL);
    //templatesStart_->setBitList (new unsigned char[numOfCharsPerElement_*numOfElementsPerTemplate_]);
    templatesStart_->setBitList (reinterpret_cast<unsigned char*>(_mm_malloc(sizeof(unsigned char)*numOfCharsPerElement_*numOfElementsPerTemplate_, 16)));
    templatesStart_->setOcclusionAmount (occlusionAmount);
    templatesStart_->setMaxResponse (maxResponse);
    templatesStart_->setROI(roi);
    
    memcpy (templatesStart_->getBitList (), bitList, numOfCharsPerElement_*numOfElementsPerTemplate_);
    
    ++numOfLearnedTemplates_;
  }
  else
  {
    List * currentList = templatesStart_;
    
    while (currentList->getNext () != NULL)
    {
      currentList = currentList->getNext ();
    }
    
    ::std::cerr << "maxResponse: " << maxResponse << ::std::endl;
    
    List * newList = new List ();
    newList->setIndex (currentList->getIndex () + 1);
    newList->setClusterIndex (0);
    newList->setClassIndex (classId);
    newList->setNext (NULL);
    //newList->setBitList (new unsigned char[numOfCharsPerElement_*numOfElementsPerTemplate_]);
    newList->setBitList (reinterpret_cast<unsigned char*>(_mm_malloc(sizeof(unsigned char)*numOfCharsPerElement_*numOfElementsPerTemplate_, 16)));
    newList->setOcclusionAmount (occlusionAmount);
    newList->setMaxResponse (maxResponse);
    newList->setROI(roi);
    
    memcpy (newList->getBitList (), bitList, numOfCharsPerElement_*numOfElementsPerTemplate_);
    
    currentList->setNext (newList);
    ++numOfLearnedTemplates_;
  }
}


// ============================================================================


void
ftd::
FastTemplateDetectorVS::
shiftInit (
  unsigned char * image,
  unsigned char * pix,
  const int width,
  const int height )
{
  unsigned char * imagePointer = image;
  unsigned char * pixPointer = pix;
  
  for (int rowIndex = 0; rowIndex < numOfVerticalSamples_; ++rowIndex)
  {
    memcpy (pixPointer, imagePointer, numOfCharsPerElement_*numOfHorizontalSamples_);
    
    pixPointer += numOfCharsPerElement_*numOfHorizontalSamples_;
    imagePointer += numOfCharsPerElement_*width;
  }
}


// ============================================================================


/*void
ftd::
FastTemplateDetectorVS::
shiftInit (
  unsigned char * image,
  const int imageWidth,
  const int imageHeight,
  unsigned char * subregion,
  const int subregionWidth,
  const int subregionHeight )
{
  unsigned char * imagePointer = image;
  unsigned char * subregionPointer = subregion;
  
  for (int rowIndex = 0; rowIndex < subregionHeight; ++rowIndex)
  {
    memcpy (subregionPointer, imagePointer, numOfCharsPerElement_*subregionWidth);
    
    subregionPointer += numOfCharsPerElement_*subregionWidth;
    imagePointer += numOfCharsPerElement_*imageWidth;
  }
}*/


// ============================================================================


void
ftd::
FastTemplateDetectorVS::
shiftRight (
  unsigned char * image,
  unsigned char * list,
  const int width,
  const int row,
  const int col )
{
  int offset2 = numOfCharsPerElement_*col + numOfCharsPerElement_*numOfHorizontalSamples_ + numOfCharsPerElement_*width*row;
  int offset1 = numOfCharsPerElement_*(numOfHorizontalSamples_-1);

  //memcpy (list, list+1, numOfElementsPerTemplate_-1);
  for (int index = 0; index < (numOfElementsPerTemplate_-1); ++index)
  {
    for (int charIndex = 0; charIndex < numOfCharsPerElement_; ++charIndex)
    {
      list[index*numOfCharsPerElement_+charIndex] = list[numOfCharsPerElement_*(index+1)+charIndex];
    }
  }

  for (int rowIndex = 0; rowIndex < numOfVerticalSamples_; ++rowIndex)
  {
    for (int charIndex = 0; charIndex < numOfCharsPerElement_; ++charIndex)
    {
      list[offset1+charIndex] = image[offset2+charIndex];
    }
    
    offset2 += numOfCharsPerElement_*width,
    offset1 += numOfCharsPerElement_*numOfHorizontalSamples_;
  }
  
  
  
//	int l_off2 = col+numOfHorizontalSamples_+width*row;
//	int l_off2 = col+width*row;
//	int l_off1 = numOfHorizontalSamples_-1;

  //unsigned char * tmp = unsigned char[numOfElementsPerTemplate_];
	//ippsCopy_8u(list+1,list,numOfElementsPerTemplate_-1);
//  for (int index = 0; index < numOfElementsPerTemplate_-1; ++index)
//  {
//    list[index] = list[index+1];
//  }

//	for( int l_i=0; l_i<numOfVerticalSamples_; ++l_i )
//	{
//	  if (l_off2 >= imageSize)
//	  {
//	    break;
//	  }
		
//		list[l_off1] = image[l_off2];

//		l_off2+=width;
//		l_off1+=numOfHorizontalSamples_;
//	}
  
}


// ============================================================================


/*void
ftd::
FastTemplateDetectorVS::
shiftRight (
  unsigned char * image,
  const int imageWidth,
  const int imageRow,
  const int imageCol,
  unsigned char * subregion,
  const int subregionWidth,
  const int subregionHeight,
  const int elementsInSubregion )
{
  int imageOffset = numOfCharsPerElement_*imageCol + numOfCharsPerElement_*subregionWidth + numOfCharsPerElement_*imageWidth*imageRow;
  int subregionOffset = subregionWidth-1;

  //memcpy (list, list+1, numOfElementsPerTemplate_-1);
  for (int index = 0; index < elementsInSubregion-numOfCharsPerElement_; ++index)
  {
    subregion[index] = subregion[index+numOfCharsPerElement_];
  }

  for (int rowIndex = 0; rowIndex < subregionHeight; ++rowIndex)
  {
    for (int charIndex = 0; charIndex < numOfCharsPerElement_; ++charIndex)
    {
      subregion[subregionOffset+charIndex] = image[imageOffset+charIndex];
    }
    
    imageOffset += numOfCharsPerElement_*imageWidth,
    subregionOffset += numOfCharsPerElement_*subregionWidth;
  }
}*/


// ============================================================================


void
ftd::
FastTemplateDetectorVS::
shiftDown (
  unsigned char * image,
  unsigned char * list,
  const int width,
  const int row,
  const int col )
{
  int offset = numOfCharsPerElement_*col + numOfCharsPerElement_*width*(row+numOfVerticalSamples_);
  int start = numOfCharsPerElement_*numOfHorizontalSamples_*(numOfVerticalSamples_-1);
  int end = start + numOfCharsPerElement_*numOfHorizontalSamples_;
  
  memcpy (list, list+numOfCharsPerElement_*numOfHorizontalSamples_, numOfCharsPerElement_*(numOfElementsPerTemplate_-numOfHorizontalSamples_));
  
  for (int rowIndex = start; rowIndex < end; rowIndex+=numOfCharsPerElement_)
  {
    for (int charIndex = 0; charIndex < numOfCharsPerElement_; ++charIndex)
    {
      list[rowIndex+charIndex] = image[offset+charIndex];
    }
    
    offset+=numOfCharsPerElement_;
  }
}


// ============================================================================


/*void
ftd::
FastTemplateDetectorVS::
shiftDown (
  unsigned char * image,
  const int imageWidth,
  const int imageCol,
  const int imageRow,
  unsigned char * subregion,
  const int subregionWidth,
  const int subregionHeight,
  const int elementsInSubregion )
{
  int offset = numOfCharsPerElement_*imageCol + numOfCharsPerElement_*imageWidth*(imageRow+subregionHeight);
  int start = numOfCharsPerElement_*subregionWidth*(subregionHeight-1);
  int end = start + numOfCharsPerElement_*subregionWidth;
  
  memcpy (subregion, subregion+numOfCharsPerElement_*subregionWidth, elementsInSubregion-numOfCharsPerElement_*subregionWidth);
  
  for (int rowIndex = start; rowIndex < end; ++rowIndex)
  {
    subregion[rowIndex] = image[offset];
    
    ++offset;
  }
}*/


// ============================================================================


void
ftd::
FastTemplateDetectorVS::
shiftCopy (
  unsigned char * oldData,
  unsigned char * newData )
{
  memcpy (newData, oldData, numOfCharsPerElement_*numOfHorizontalSamples_*numOfVerticalSamples_);
}


// ============================================================================


/*void
ftd::
FastTemplateDetectorVS::
shiftCopy (
  unsigned char * oldData,
  unsigned char * newData,
  const int numOfElements )
{
  memcpy (newData, oldData, numOfElements);
}*/


// ============================================================================


/*void
ftd::
FastTemplateDetectorVS::
copyData (
  unsigned char * image,
  const int imageWidth,
  const int imageHeight,
  const int imageCol,
  const int imageRow,
  unsigned char * subregion,
  const int subregionWidth,
  const int subregionHeight )
{
  unsigned char * imagePointer = image + imageRow*numOfCharsPerElement_*imageWidth + numOfCharsPerElement_*imageCol;
  unsigned char * subregionPointer = subregion;
  
  for (int rowIndex = 0; rowIndex < subregionHeight; ++rowIndex)
  {
    memcpy (subregionPointer, imagePointer, numOfCharsPerElement_*subregionWidth);
  
    imagePointer += numOfCharsPerElement_*imageWidth;
    subregionPointer += numOfCharsPerElement_*subregionWidth;
  }
}*/


// ============================================================================


::ftd::Candidate *
ftd::
FastTemplateDetectorVS::
computeCandidate (
  unsigned char * gradientData, 
  const int threshold )
{
  using ::ftd::Candidate;
  
  /*register*/ __m128i zero = _mm_setzero_si128();
  
#ifndef DOT_NO_CLUSTERING
  Candidate * candidate = new Candidate();
  candidate->setIndex (0);
  candidate->setClusterIndex (0);
  candidate->setClassIndex (0);
  candidate->setMatchingResponse (0);
    
  unsigned short maxResponse = 0;
  
  List * currentCluster = clusterStart_;
  
  while (currentCluster != NULL)
  {
    const int clusterResponse = this->energyBitCountLayerVS (currentCluster->getSTA (), gradientData, zero, (numOfCharsPerElement_*numOfHorizontalSamples_*numOfVerticalSamples_-1)/16+1);
    
    if ( clusterResponse >= threshold
      && clusterResponse > maxResponse )
    {
      List * currentTemplate = currentCluster->getFLW ();
      
      while (currentTemplate != NULL)
      {
        const int templateResponse = this->energyBitCountLayerVS (currentTemplate->getSTA (), gradientData, zero, (numOfCharsPerElement_*numOfHorizontalSamples_*numOfVerticalSamples_-1)/16+1);
        
        if ( templateResponse > maxResponse
          && templateResponse >= threshold )
        {
          maxResponse = templateResponse;
          
          candidate->setIndex (currentTemplate->getIndex ());
          candidate->setClusterIndex (currentTemplate->getClusterIndex ());
          candidate->setClassIndex (currentTemplate->getClassIndex ());
          candidate->setMatchingResponse (templateResponse);
      
          CvRect roi = currentTemplate->getROI ();
          candidate->setROI (roi);
        }
        
        currentTemplate = currentTemplate->getFLW ();
      }
    }
    
    currentCluster = currentCluster->getNext ();
  }
  
#else // ifndef DOT_NO_CLUSTERING
  Candidate * candidate = new Candidate();
  candidate->setIndex (0);
  candidate->setClusterIndex (0);
  candidate->setClassIndex (0);
  candidate->setMatchingResponse (0);
  
  unsigned short maxResponse = 0;
  
  List * currentTemplate = templatesStart_;
  
  while (currentTemplate != NULL)
  {
    //std::cerr << currentTemplate->getIndex () << ",";
    const unsigned short currentResponse = this->energyBitCountLayerVS (currentTemplate->getBitList (), gradientData, zero, (numOfCharsPerElement_*numOfHorizontalSamples_*numOfVerticalSamples_-1)/16+1);
    
    const int maxTemplateResponse = currentTemplate->getMaxResponse ();
    const int relativeResponse = (currentResponse*100)/maxTemplateResponse;
    
    //std::cerr << "(" << currentResponse << "," << maxTemplateResponse << "):" << relativeResponse << ",";
        
    if ( relativeResponse >= threshold
      && relativeResponse > maxResponse )
    {
      candidate->setIndex (currentTemplate->getIndex ());
      candidate->setClusterIndex (currentTemplate->getClusterIndex ());
      candidate->setClassIndex (currentTemplate->getClassIndex ());
      candidate->setMatchingResponse (relativeResponse);
      
      CvRect roi = currentTemplate->getROI ();
      candidate->setROI (roi);

      maxResponse = relativeResponse;
      
      //std::cerr << relativeResponse << ",";
    }
    
    currentTemplate = currentTemplate->getNext ();
  }
#endif // ifndef DOT_NO_CLUSTERING
   
  return candidate;
}


// ============================================================================


void
ftd::
FastTemplateDetectorVS::
computeCandidates (
  unsigned char * gradientData, 
  const int threshold,
  std::list< ::ftd::Candidate* > * candidateList,
  const int col,
  const int row )
{
  using ::ftd::Candidate;
  
  /*register*/ __m128i zero = _mm_setzero_si128();
  
#ifndef DOT_NO_CLUSTERING
  List * currentCluster = clusterStart_;
  
  while (currentCluster != NULL)
  {
    const int clusterResponse = this->energyBitCountLayerVS (currentCluster->getSTA (), gradientData, zero, (numOfCharsPerElement_*numOfHorizontalSamples_*numOfVerticalSamples_-1)/16+1);
    
    if ( clusterResponse >= threshold )
    {
      List * currentTemplate = currentCluster->getFLW ();
      
      while (currentTemplate != NULL)
      {
        const int templateResponse = this->energyBitCountLayerVS (currentTemplate->getSTA (), gradientData, zero, (numOfCharsPerElement_*numOfHorizontalSamples_*numOfVerticalSamples_-1)/16+1);
        
        if ( templateResponse >= threshold )
        {
          Candidate * candidate = new Candidate();
          candidate->setIndex (currentTemplate->getIndex ());
          candidate->setClusterIndex (currentTemplate->getClusterIndex ());
          candidate->setClassIndex (currentTemplate->getClassIndex ());
          candidate->setMatchingResponse (templateResponse);
          
          candidate->setCol (col);
          candidate->setRow (row);
          
          candidateList[currentTemplate->getClassIndex ()].push_back (candidate);
        }
        
        currentTemplate = currentTemplate->getFLW ();
      }
    }
    
    currentCluster = currentCluster->getNext ();
  }
  
#else // ifndef DOT_NO_CLUSTERING
  List * currentTemplate = templatesStart_;
  
  while (currentTemplate != NULL)
  {
    const unsigned short currentResponse = this->energyBitCountLayerVS (currentTemplate->getBitList (), gradientData, zero, (numOfCharsPerElement_*numOfHorizontalSamples_*numOfVerticalSamples_-1)/16+1);
    
    if ( currentResponse >= threshold )
    {
      Candidate * candidate = new Candidate();
      candidate->setIndex (currentTemplate->getIndex ());
      candidate->setClusterIndex (currentTemplate->getClusterIndex ());
      candidate->setClassIndex (currentTemplate->getClassIndex ());
      candidate->setMatchingResponse (currentResponse);
      
      candidate->setCol (col);
      candidate->setRow (row);

      candidateList[currentTemplate->getClassIndex ()].push_back (candidate);
    }
    
    currentTemplate = currentTemplate->getNext ();
  }
#endif // ifndef DOT_NO_CLUSTERING
}


// ============================================================================


::ftd::Candidate *
ftd::
FastTemplateDetectorVS::
computeCandidate (
  unsigned char * gradientData, 
  const int threshold,
  const int occlusionAmount )
{
  using ::ftd::Candidate;
  
  /*register*/ __m128i zero = _mm_setzero_si128();
  
/*#ifndef DOT_NO_CLUSTERING
  Candidate * candidate = new Candidate();
  candidate->setIndex (0);
  candidate->setClusterIndex (0);
  candidate->setClassIndex (0);
  candidate->setMatchingResponse (0);
    
  unsigned short maxResponse = 0;
  
  List * currentCluster = clusterStart_;
  
  while (currentCluster != NULL)
  {
    const int clusterResponse = this->energyBitCountLayerVS (currentCluster->getSTA (), gradientData, zero, (numOfCharsPerElement_*numOfHorizontalSamples_*numOfVerticalSamples_-1)/16+1);
    
    if ( clusterResponse >= threshold
      && clusterResponse > maxResponse )
    {
      List * currentTemplate = currentCluster->getFLW ();
      
      while (currentTemplate != NULL)
      {
        const int templateResponse = this->energyBitCountLayerVS (currentTemplate->getSTA (), gradientData, zero, (numOfCharsPerElement_*numOfHorizontalSamples_*numOfVerticalSamples_-1)/16+1);
        
        if ( templateResponse > maxResponse
          && templateResponse >= threshold )
        {
          maxResponse = templateResponse;
          
          candidate->setIndex (currentTemplate->getIndex ());
          candidate->setClusterIndex (currentTemplate->getClusterIndex ());
          candidate->setClassIndex (currentTemplate->getClassIndex ());
          candidate->setMatchingResponse (templateResponse);
        }
        
        currentTemplate = currentTemplate->getFLW ();
      }
    }
    
    currentCluster = currentCluster->getNext ();
  }
  
#else // ifndef DOT_NO_CLUSTERING */
  Candidate * candidate = new Candidate();
  candidate->setIndex (0);
  candidate->setClusterIndex (0);
  candidate->setClassIndex (0);
  candidate->setMatchingResponse (0);
  
  unsigned short maxResponse = 0;
  
  List * currentTemplate = templatesStart_;
  
  while (currentTemplate != NULL)
  {
    const unsigned short currentResponse = this->energyBitCountLayerVS (currentTemplate->getBitList (), gradientData, zero, (numOfCharsPerElement_*numOfHorizontalSamples_*numOfVerticalSamples_-1)/16+1);
    
    const int maxValue = numOfHorizontalSamples_*numOfVerticalSamples_ - std::max (occlusionAmount, currentTemplate->getOcclusionAmount ());
    int newResponse = static_cast<int>(100.0f*static_cast<float>(currentResponse)/static_cast<float>(maxValue));
    newResponse += static_cast<int>(100.0f*static_cast<float>(currentResponse)/static_cast<float>(numOfHorizontalSamples_*numOfVerticalSamples_ - currentTemplate->getOcclusionAmount ()));
    newResponse /= 2;    
            
    if ( newResponse >= threshold
      && newResponse > maxResponse )
    {
      candidate->setIndex (currentTemplate->getIndex ());
      candidate->setClusterIndex (currentTemplate->getClusterIndex ());
      candidate->setClassIndex (currentTemplate->getClassIndex ());
      candidate->setMatchingResponse (newResponse);

      maxResponse = newResponse;
    }
    
    currentTemplate = currentTemplate->getNext ();
  }
//#endif // ifndef DOT_NO_CLUSTERING
   
  return candidate;
}


// ============================================================================


::ftd::Candidate *
ftd::
FastTemplateDetectorVS::
computeCandidateMasked (
  unsigned char * gradientData, 
  const int threshold,
  const int occlusionAmount )
{
  using ::ftd::Candidate;
  
  /*register*/ __m128i zero = _mm_setzero_si128();
  
/*#ifndef DOT_NO_CLUSTERING
  Candidate * candidate = new Candidate();
  candidate->setIndex (0);
  candidate->setClusterIndex (0);
  candidate->setClassIndex (0);
  candidate->setMatchingResponse (0);
    
  unsigned short maxResponse = 0;
  
  List * currentCluster = clusterStart_;
  
  while (currentCluster != NULL)
  {
    const int clusterResponse = this->energyBitCountLayerVS (currentCluster->getSTA (), gradientData, zero, (numOfCharsPerElement_*numOfHorizontalSamples_*numOfVerticalSamples_-1)/16+1);
    
    if ( clusterResponse >= threshold
      && clusterResponse > maxResponse )
    {
      List * currentTemplate = currentCluster->getFLW ();
      
      while (currentTemplate != NULL)
      {
        const int templateResponse = this->energyBitCountLayerVS (currentTemplate->getSTA (), gradientData, zero, (numOfCharsPerElement_*numOfHorizontalSamples_*numOfVerticalSamples_-1)/16+1);
        
        if ( templateResponse > maxResponse
          && templateResponse >= threshold )
        {
          maxResponse = templateResponse;
          
          candidate->setIndex (currentTemplate->getIndex ());
          candidate->setClusterIndex (currentTemplate->getClusterIndex ());
          candidate->setClassIndex (currentTemplate->getClassIndex ());
          candidate->setMatchingResponse (templateResponse);
        }
        
        currentTemplate = currentTemplate->getFLW ();
      }
    }
    
    currentCluster = currentCluster->getNext ();
  }
  
#else // ifndef DOT_NO_CLUSTERING */
  Candidate * candidate = new Candidate();
  candidate->setIndex (0);
  candidate->setClusterIndex (0);
  candidate->setClassIndex (0);
  candidate->setMatchingResponse (0);
  
  unsigned short maxResponse = 0;
  
  List * currentTemplate = templatesStart_;
  
  while (currentTemplate != NULL)
  {
    const unsigned short currentResponse = this->energyBitCountLayerVS (currentTemplate->getBitList (), gradientData, zero, (numOfCharsPerElement_*numOfHorizontalSamples_*numOfVerticalSamples_-1)/16+1);
    
    //const int maxValue = numOfHorizontalSamples_*numOfVerticalSamples_ - std::max (occlusionAmount, currentTemplate->getOcclusionAmount ());
    //int newResponse = static_cast<int>(100.0f*static_cast<float>(currentResponse)/static_cast<float>(maxValue));
    //newResponse += static_cast<int>(100.0f*static_cast<float>(currentResponse)/static_cast<float>(numOfHorizontalSamples_*numOfVerticalSamples_ - currentTemplate->getOcclusionAmount ()));
    //newResponse /= 2;    
    int newResponse = (currentResponse*100)/currentTemplate->getMaxResponse ();
    
    //std::cerr << "currentResponse: " << currentResponse << " , maxResponse: " << currentTemplate->getMaxResponse () << std::endl;
            
    if ( newResponse >= threshold
      && newResponse > maxResponse )
    {
      candidate->setIndex (currentTemplate->getIndex ());
      candidate->setClusterIndex (currentTemplate->getClusterIndex ());
      candidate->setClassIndex (currentTemplate->getClassIndex ());
      candidate->setMatchingResponse (newResponse);

      maxResponse = newResponse;
    }
    
    currentTemplate = currentTemplate->getNext ();
  }
//#endif // ifndef DOT_NO_CLUSTERING
   
  return candidate;
}


// ============================================================================


/*::ftd::Candidate *
ftd::
FastTemplateDetectorVS::
computeCandidate (
  unsigned char * gradientData, 
  const int threshold,
  const int layerIndex,
  const int templateWidth,
  const int templateHeight )
{
  using ::ftd::Candidate;
  
  __m128i zero = _mm_setzero_si128();
  
#ifndef DOT_NO_CLUSTERING
  Candidate * candidate = new Candidate();
  candidate->setIndex (0);
  candidate->setClusterIndex (0);
  candidate->setClassIndex (0);
  candidate->setMatchingResponse (0);
  
  std::cerr << "ERROR: DOT_NO_CLUSTERING not defined" << std::endl;
  
#else // ifndef DOT_NO_CLUSTERING
  Candidate * candidate = new Candidate();
  candidate->setIndex (0);
  candidate->setClusterIndex (0);
  candidate->setClassIndex (0);
  candidate->setMatchingResponse (0);
  
  unsigned short maxResponse = 0;
  
  List * currentTemplate = templatesStart_;
  
  while (currentTemplate != NULL)
  {
    //std::cerr << "pyramid size: " << currentTemplate->pyramid.size() << std::endl;
    const unsigned short currentResponse = pow(4, layerIndex+1) * energyBitCountLayerVS (currentTemplate->pyramid[layerIndex], gradientData, zero, (templateWidth*templateHeight-1)/16+1);
    
    if ( currentResponse >= threshold
      && currentResponse > maxResponse )
    {
      candidate->setIndex (currentTemplate->getIndex ());
      candidate->setClusterIndex (currentTemplate->getClusterIndex ());
      candidate->setClassIndex (currentTemplate->getClassIndex ());
      candidate->setMatchingResponse (currentResponse);

      maxResponse = currentResponse;
    }
    
    currentTemplate = currentTemplate->getNext ();
  }
#endif // ifndef DOT_NO_CLUSTERING
   
  return candidate;
}*/


// ============================================================================


/*void
ftd::
FastTemplateDetectorVS::
computeCandidates (
  unsigned char * gradientData, 
  const int threshold,
  const int layerIndex,
  const int templateWidth,
  const int templateHeight,
  std::list< ::ftd::Candidate * > * candidates,
  const int colIndex,
  const int rowIndex )
{
  using ::ftd::Candidate;
  
  __m128i zero = _mm_setzero_si128();
  
#ifndef DOT_NO_CLUSTERING

  std::cerr << "ERROR: DOT_NO_CLUSTERING not defined" << std::endl;
  
#else // ifndef DOT_NO_CLUSTERING

  List * currentTemplate = templatesStart_;
  
  while (currentTemplate != NULL)
  {
    const unsigned short currentResponse = pow(4, layerIndex+1) * energyBitCountLayerVS (currentTemplate->pyramid[layerIndex], gradientData, zero, (templateWidth*templateHeight-1)/16+1);
    
    if ( currentResponse >= threshold )
    {
      Candidate * candidate = new Candidate();
      candidate->setIndex (currentTemplate->getIndex ());
      candidate->setClusterIndex (currentTemplate->getClusterIndex ());
      candidate->setClassIndex (currentTemplate->getClassIndex ());
      candidate->setMatchingResponse (currentResponse);
      candidate->setCol (colIndex);
      candidate->setRow (rowIndex);
      
      candidates[currentTemplate->getClassIndex ()].push_back (candidate);
    }
    
    currentTemplate = currentTemplate->getNext ();
  }
#endif // ifndef DOT_NO_CLUSTERING
}*/


// ============================================================================


void
ftd::
FastTemplateDetectorVS::
transferBitList (
  unsigned char * memory )
{
  using ::ftd::List;
  
  unsigned char * endPointer = memory_ + numOfCharsPerElement_*numOfElementsPerTemplate_*MAX_NUM_OF_DOT_TEMPLATES;
  
#ifndef DOT_NO_CLUSTERING
  unsigned char * memoryPointer = memory_;
  
  List * clusterPointer = clusterStart_;
  
  while (clusterPointer != NULL)
  {
    if (memoryPointer == endPointer)
    {
      std::cerr << "error: out of memory..." << std::endl;
      return;
    }
    
    //memcpy (memoryPointer, clusterPointer->getBitList (), numOfElementsPerTemplate_);
    unsigned char * clusterPointerBitList = clusterPointer->getBitList ();
    for (int index = 0; index < numOfCharsPerElement_*numOfElementsPerTemplate_; ++index)
    {
      memoryPointer[index] = clusterPointerBitList[index];
    }
    clusterPointer->setSTA (memoryPointer);
    memoryPointer += numOfCharsPerElement_*numOfElementsPerTemplate_;
    
    List * currentPointer = clusterPointer->getFLW ();
    
    while (currentPointer != NULL)
    {
      if (memoryPointer == endPointer)
      {
        std::cerr << "error: out of memory..." << std::endl;
        return;
      }
      
      //memcpy (memoryPointer, currentPointer->getBitList (), numOfElementsPerTemplate_);
      unsigned char * currentPointerBitList = currentPointer->getBitList ();
      for (int index = 0; index < numOfCharsPerElement_*numOfElementsPerTemplate_; ++index)
      {
        memoryPointer[index] = currentPointerBitList[index];
      }
      currentPointer->setSTA (memoryPointer);
      memoryPointer += numOfCharsPerElement_*numOfElementsPerTemplate_;
      
      currentPointer = currentPointer->getFLW ();
    }
    
    clusterPointer = clusterPointer->getNext ();
  }
#else // ifndef DOT_NO_CLUSTERING
  unsigned char * memoryPointer = memory_;
  
  List * currentPointer = templatesStart_;
  
  while (currentPointer != NULL)
  {
    if (memoryPointer == endPointer)
    {
      std::cerr << "error: out of memory..." << std::endl;
      return;
    }
    
    //memcpy (memoryPointer, currentPointer->getBitList (), numOfElementsPerTemplate_);
    unsigned char * currentPointerBitList = currentPointer->getBitList ();
    for (int index = 0; index < numOfCharsPerElement_*numOfElementsPerTemplate_; ++index)
    {
      memoryPointer[index] = currentPointerBitList[index];
    }
    currentPointer->setSTA (memoryPointer);
    memoryPointer += numOfCharsPerElement_*numOfElementsPerTemplate_;
    
    currentPointer = currentPointer->getNext ();
  }
#endif // ifndef DOT_NO_CLUSTERING

  return;
}


// ============================================================================


unsigned short
ftd::
FastTemplateDetectorVS::
clusterBitCount (
  unsigned char * pix1,
  unsigned char * pix2 )
{
  int value1 = 0;
  int value2 = 0;
  int value3 = 0;
  
  unsigned char * result = new unsigned char[numOfCharsPerElement_*numOfElementsPerTemplate_];
  
  for (int index = 0; index < numOfCharsPerElement_*numOfElementsPerTemplate_; ++index)
  {
    value1 += bitsSetIn16bitVS_[pix1[index]];
    value2 += bitsSetIn16bitVS_[pix2[index]];
  }
  this->computeOR(pix1, pix2, result);
  
  for (int index = 0; index < numOfElementsPerTemplate_; ++index)
  {
    value3 += bitsSetIn16bitVS_[result[index]];  
  }
  
  delete[] result;
  
  return std::max (value3-value1, value3-value2);
}

        
// ============================================================================


unsigned short
ftd::
FastTemplateDetectorVS::
bitSetsBitCount (
  unsigned char * pix1 )
{
  unsigned short value1 = 0;
  
  for (int index = 0; index < numOfCharsPerElement_*numOfElementsPerTemplate_; ++index)
  {
    value1 += bitsSetIn16bitVS_[pix1[index]];
  }
  
  return value1;
}


// ============================================================================


void
ftd::
FastTemplateDetectorVS::
computeOR (
  unsigned char * pix1,
  unsigned char * pix2,
  unsigned char * result )
{
  __m128i * resultPointer = reinterpret_cast<__m128i*> (result);
  __m128i * pix1Pointer = reinterpret_cast<__m128i*> (pix1);
  __m128i * pix2Pointer = reinterpret_cast<__m128i*> (pix2);
  
  for (int index = 0; index < (numOfCharsPerElement_*numOfElementsPerTemplate_)/16; ++index)
  {
    resultPointer[index] = _mm_or_si128 (pix1Pointer[index], pix2Pointer[index]);
  }
}


// ============================================================================


std::ofstream & 
ftd::
FastTemplateDetectorVS::
write (
  std::ofstream & stream )
{
  using ::ftd::List;
  
  stream.write (reinterpret_cast<char*> (&numOfLearnedTemplates_), sizeof (numOfLearnedTemplates_));
  stream.write (reinterpret_cast<char*> (&numOfLearnedClasses_), sizeof (numOfLearnedClasses_));
  stream.write (reinterpret_cast<char*> (&areTemplatesClustered_), sizeof (areTemplatesClustered_));
  stream.write (reinterpret_cast<char*> (&currentImageWidth_), sizeof (currentImageWidth_));
  stream.write (reinterpret_cast<char*> (&currentImageHeight_), sizeof (currentImageHeight_));
  stream.write (reinterpret_cast<char*> (&numOfPyramidLevels_), sizeof (numOfPyramidLevels_));
  stream.write (reinterpret_cast<char*> (&numOfHorizontalSamples_), sizeof (numOfHorizontalSamples_));
  stream.write (reinterpret_cast<char*> (&numOfVerticalSamples_), sizeof (numOfVerticalSamples_));
  stream.write (reinterpret_cast<char*> (&samplingSize_), sizeof (samplingSize_));
  stream.write (reinterpret_cast<char*> (&numOfCharsPerElement_), sizeof (numOfCharsPerElement_));

  
  List * currentTemplate = templatesStart_;
  
  while (currentTemplate != NULL)
  {
    int classIndex = currentTemplate->getClassIndex ();
    
    stream.write (reinterpret_cast<char*> (&classIndex), sizeof (classIndex));
    
    for (int index = 0; index < numOfCharsPerElement_*numOfElementsPerTemplate_; ++index)
    {
      stream.write (reinterpret_cast<char*> (&(currentTemplate->getBitList ()[index])), sizeof (currentTemplate->getBitList ()[index]));
    }

    int maxResponse = currentTemplate->getMaxResponse ();
    stream.write (reinterpret_cast<char*> (&maxResponse), sizeof (maxResponse));
    
    CvRect roi = currentTemplate->getROI ();
    stream.write (reinterpret_cast<char*> (&roi), sizeof (roi));
    
    currentTemplate = currentTemplate->getNext ();
  }
  
  int numOfContours = contours_.size ();
  stream.write (reinterpret_cast<char*> (&numOfContours), sizeof (numOfContours));

  for (int contourIndex = 0; contourIndex < numOfContours; ++contourIndex)
  {
    write (stream, contours_[contourIndex]);
  }
  
  return stream;
}


// ============================================================================


std::ifstream & 
ftd::
FastTemplateDetectorVS::
read (
  std::ifstream & stream )
{
  this->clearClusters ();
  this->clearBitLists ();

  
  stream.read (reinterpret_cast<char*> (&numOfLearnedTemplates_), sizeof (numOfLearnedTemplates_));
  stream.read (reinterpret_cast<char*> (&numOfLearnedClasses_), sizeof (numOfLearnedClasses_));
  stream.read (reinterpret_cast<char*> (&areTemplatesClustered_), sizeof (areTemplatesClustered_));
  stream.read (reinterpret_cast<char*> (&currentImageWidth_), sizeof (currentImageWidth_));
  stream.read (reinterpret_cast<char*> (&currentImageHeight_), sizeof (currentImageHeight_));
  stream.read (reinterpret_cast<char*> (&numOfPyramidLevels_), sizeof (numOfPyramidLevels_));
  stream.read (reinterpret_cast<char*> (&numOfHorizontalSamples_), sizeof (numOfHorizontalSamples_));
  stream.read (reinterpret_cast<char*> (&numOfVerticalSamples_), sizeof (numOfVerticalSamples_));
  stream.read (reinterpret_cast<char*> (&samplingSize_), sizeof (samplingSize_));
  stream.read (reinterpret_cast<char*> (&numOfCharsPerElement_), sizeof (numOfCharsPerElement_));
  
  const int size = numOfLearnedTemplates_;
  
  numOfLearnedTemplates_ = 0; // will be automatically increased while adding data from stream
  
  
  for (int templateIndex = 0; templateIndex < size; ++templateIndex)
  {
    int classIndex;
    stream.read (reinterpret_cast<char*> (&classIndex), sizeof (classIndex));
    
    //unsigned char * bitList = new unsigned char[numOfCharsPerElement_*numOfElementsPerTemplate_];    
    unsigned char * bitList = reinterpret_cast<unsigned char*>(_mm_malloc(sizeof(unsigned char)*numOfCharsPerElement_*numOfElementsPerTemplate_, 16));
    
    for (int elementIndex = 0; elementIndex < numOfCharsPerElement_*numOfElementsPerTemplate_; ++elementIndex)
    {
      stream.read (reinterpret_cast<char*> (&(bitList[elementIndex])), sizeof (bitList[elementIndex]));
    }
    
    int maxResponse = -1;
    stream.read (reinterpret_cast<char*> (&(maxResponse)), sizeof (maxResponse));
    
    CvRect roi;
    stream.read (reinterpret_cast<char*> (&(roi)), sizeof (roi));
    
    this->addBitList (bitList, classIndex, maxResponse, 0, roi);
//    this->addBitList (bitList, classIndex, maxResponse);
    
    //delete[] bitList;
    _mm_free(bitList);
  }

  int numOfContours = 0;
  stream.read (reinterpret_cast<char*> (&numOfContours), sizeof (numOfContours));

  for (unsigned int contourIndex = 0; contourIndex < numOfContours; ++contourIndex)
  {
    IplImage * contour = readImage (stream);
    
    contours_.push_back (contour);
  }  
  
  return stream;
}


// ============================================================================


void
ftd::
FastTemplateDetectorVS::
combine (
  FastTemplateDetectorVS & detector )
{
  int classCount = this->getNumOfClasses ();
  
  for (int classIndex = 0; classIndex < detector.getNumOfClasses (); ++classIndex)
  {
    this->addNewClass ();
  }
  
  List * currentElement = detector.templatesStart_;
  
  while (currentElement != NULL)
  {
    this->addBitList (currentElement->getBitList (), currentElement->getClassIndex () + classCount, currentElement->getMaxResponse (), 0, currentElement->getROI ());
    currentElement = currentElement->getNext ();
  }
  
  for (unsigned int contourIndex = 0; contourIndex < detector.contours_.size (); ++contourIndex)
  {
    contours_.push_back (detector.contours_[contourIndex]);
  }
}


// ============================================================================


int
ftd::
FastTemplateDetectorVS::
computeTemplateResponse (
  unsigned char * templateData,
  const int templateId )
{
  unsigned short response = 0;
  
  __m128i zero = _mm_setzero_si128();
  
  List * currentTemplate = templatesStart_;
  
  while (currentTemplate != NULL)
  {
    if (currentTemplate->getIndex () == templateId)
    {
      const unsigned short currentResponse = this->energyBitCountLayerVS (currentTemplate->getBitList (), templateData, zero, (numOfCharsPerElement_*numOfHorizontalSamples_*numOfVerticalSamples_-1)/16+1);
      
      const int maxTemplateResponse = currentTemplate->getMaxResponse ();
      response = (currentResponse*100)/maxTemplateResponse;
      
      break;
    }
    
    currentTemplate = currentTemplate->getNext ();
  }  
  
  return static_cast<int> (response);
}


// ============================================================================


void
ftd::
FastTemplateDetectorVS::
addContour (
  IplImage * templateImage )
{
  // compute gradients
  IplImage * sobelDx = cvCreateImage (cvGetSize (templateImage), IPL_DEPTH_32F, 1);
  IplImage * sobelDy = cvCreateImage (cvGetSize (templateImage), IPL_DEPTH_32F, 1);
  IplImage * sobelMagnitude = cvCreateImage (cvGetSize (templateImage), IPL_DEPTH_32F, 1);
  IplImage * sobelAngle = cvCreateImage (cvGetSize (templateImage), IPL_DEPTH_32F, 1);

  cvSobel (templateImage, sobelDx, 1, 0, 3);
  cvSobel (templateImage, sobelDy, 0, 1, 3);

  cvCartToPolar (sobelDx, sobelDy, sobelMagnitude, sobelAngle, 1); // the "1" means that the angles are in degree
  
  for (int rowIndex = 0; rowIndex < sobelMagnitude->height; ++rowIndex)
  {
    for (int colIndex = 0; colIndex < sobelMagnitude->width; ++colIndex)
    {
      if (CV_IMAGE_ELEM (sobelMagnitude, float, rowIndex, colIndex) < 10.0f)
      {
        CV_IMAGE_ELEM (sobelMagnitude, float, rowIndex, colIndex) = 0.0f;
      }
      else
      {
        CV_IMAGE_ELEM (sobelMagnitude, float, rowIndex, colIndex) = 255.0f;
      }
    }
  }
  
  cvReleaseImage (&sobelDx);
  cvReleaseImage (&sobelDy);
  cvReleaseImage (&sobelAngle);
  
  contours_.push_back (sobelMagnitude);
}


// ============================================================================


std::ofstream & 
ftd::
FastTemplateDetectorVS::
write (
  std::ofstream & stream,
  IplImage * image )
{
  int numOfRows = image->height;
  int numOfCols = image->width;
  
  stream.write (reinterpret_cast<char*> (&numOfRows), sizeof (numOfRows));
  stream.write (reinterpret_cast<char*> (&numOfCols), sizeof (numOfCols));
  stream.write (reinterpret_cast<char*> (&(image->depth)), sizeof (image->depth));
  stream.write (reinterpret_cast<char*> (&(image->nChannels)), sizeof (image->nChannels));
  
  for (int index = 0; index < (numOfRows*numOfCols*image->nChannels*image->depth)/8; ++index)
  {
    stream.write (reinterpret_cast<char*> (&(image->imageData)[index]), sizeof ((image->imageData)[index]));
  }
  
  return stream;
}



// ============================================================================


IplImage *
ftd::
FastTemplateDetectorVS::
readImage (
  std::ifstream & stream )
{
  int numOfRows;
  int numOfCols;
  int depth;
  int channels;
  
  stream.read (reinterpret_cast<char*> (&numOfRows), sizeof (numOfRows));
  stream.read (reinterpret_cast<char*> (&numOfCols), sizeof (numOfCols));
  stream.read (reinterpret_cast<char*> (&depth), sizeof (depth));
  stream.read (reinterpret_cast<char*> (&channels), sizeof (channels));
  
  IplImage * image = cvCreateImage (cvSize (numOfCols, numOfRows), depth, channels);
  for (int index = 0; index < (numOfRows*numOfCols*channels*depth/8); ++index)
  {
    stream.read (reinterpret_cast<char*> (&(image->imageData)[index]), sizeof ((image->imageData)[index]));
  }
  
  return image;
}


// ============================================================================


void
ftd::
FastTemplateDetectorVS::
drawContour (
  const int contourId,
  IplImage * image,
  const int startX,
  const int startY )
{
  IplImage * templateImage = contours_[contourId];
  
  for (int rowIndex = 0; rowIndex < templateImage->height; ++rowIndex)
  {
    for (int colIndex = 0; colIndex < templateImage->width; ++colIndex)
    {
      if (CV_IMAGE_ELEM (templateImage, float, rowIndex, colIndex) != 0)
      {
        CV_IMAGE_ELEM (image, float, rowIndex+startY, 3*(colIndex+startX)+0) = 255; 
        CV_IMAGE_ELEM (image, float, rowIndex+startY, 3*(colIndex+startX)+1) = 0; 
        CV_IMAGE_ELEM (image, float, rowIndex+startY, 3*(colIndex+startX)+2) = 0; 
      }
    }
  }
}


// ============================================================================


void
ftd::
FastTemplateDetectorVS::
drawContour (
  const int contourId,
  IplImage * image,
  const int startX,
  const int startY,
  const float scale )
{
  IplImage * templateImage = contours_[contourId];
  
  IplImage * scaledTemplateImage = cvCreateImage (cvSize (templateImage->width*scale, templateImage->height*scale), IPL_DEPTH_32F, 1);
  cvResize (templateImage, scaledTemplateImage);
  
  for (int rowIndex = 0; rowIndex < scaledTemplateImage->height; ++rowIndex)
  {
    for (int colIndex = 0; colIndex < scaledTemplateImage->width; ++colIndex)
    {
      if (CV_IMAGE_ELEM (scaledTemplateImage, float, rowIndex, colIndex) != 0)
      {
        CV_IMAGE_ELEM (image, float, rowIndex+startY, 3*(colIndex+startX)+0) = 255; 
        CV_IMAGE_ELEM (image, float, rowIndex+startY, 3*(colIndex+startX)+1) = 0; 
        CV_IMAGE_ELEM (image, float, rowIndex+startY, 3*(colIndex+startX)+2) = 0; 
      }
    }
  }
  
  cvReleaseImage (&scaledTemplateImage);
}


// ============================================================================


void
ftd::
FastTemplateDetectorVS::
removeTemplate (
  const int templateId )
{
  //cvShowImage ("contour", *(contours_.begin () + templateId));
  //cvWaitKey (-1);
  
  contours_.erase (contours_.begin () + templateId);

  List * previousTemplate = NULL;
  List * currentTemplate = templatesStart_;
  
  while (currentTemplate != NULL)
  {
    if (currentTemplate->getIndex ()-1 == templateId)
    {
      previousTemplate->setNext (currentTemplate->getNext ());
      
      unsigned char * bitList = currentTemplate->getBitList ();
      
      IplImage * temp = cvCreateImage (cvSize (numOfHorizontalSamples_, numOfVerticalSamples_), IPL_DEPTH_8U, 1);
      for (int rowIndex = 0; rowIndex < numOfVerticalSamples_; ++rowIndex)
      {
        for (int colIndex = 0; colIndex < numOfHorizontalSamples_; ++colIndex)
        {
          CV_IMAGE_ELEM (temp, unsigned char, rowIndex, colIndex) = bitList[rowIndex*numOfHorizontalSamples_+colIndex] == 0 ? 0 : 255;
        }
      }
      
      //cvShowImage ("temp", temp);
      //cvWaitKey (-1);
      
      std::cerr << "deleted " << currentTemplate->getIndex ()-1 << std::endl;

      currentTemplate = previousTemplate;
    }

    if (currentTemplate->getIndex ()-1 > templateId)
    {
      currentTemplate->setIndex (currentTemplate->getIndex ()-1);
    }
    
    previousTemplate = currentTemplate;
    currentTemplate = currentTemplate->getNext ();
  }  
  
  --numOfLearnedTemplates_;
}




