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

#ifndef FTD_UTIL_H_
#define FTD_UTIL_H_ FTD_UTIL_H_

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
   * \class 
   * \brief Candidate for a detected template.
   */
  class Candidate
  {
  
  public: // functions
  
    Candidate ()
      : index_ (0),
        clusterIndex_ (0),
        classIndex_ (0),
        matchingResponse_ (0),
        row_ (0),
        col_ (0)
    {};
    
    ~Candidate () {};
  
    int getIndex () { return index_; }
    int getClusterIndex () { return clusterIndex_; }
    int getClassIndex () { return classIndex_; }
    
    int getRow () { return row_; }
    int getCol () { return col_; }

    int getMaxResponse () { return maxResponse_; }
    void setMaxResponse (const int maxResponse) { maxResponse_ = maxResponse; }
    
    void setROI (CvRect & roi) { roi_ = roi; }
    CvRect getROI () { return roi_; }

    void setIndex (
      const int index )
    {
      index_ = index;
    }
    
    void setClusterIndex (
      const int clusterIndex )
    {
      clusterIndex_ = clusterIndex;
    }
    
    void setClassIndex (
      const int classIndex )
    {
      classIndex_ = classIndex;
    }
  
    void setRow (
      const int row )
    {
      row_ = row;
    }
  
    void setCol (
      const int col )
    {
      col_ = col;
    }
  
    /**
     * \brief Returns the matching response of the DOT candidate.
     */
    inline int 
      getMatchingResponse () 
    { 
      return matchingResponse_; 
    }
    
    /**
     * \brief Sets the matching response of the DOT candidate.
     */
    void
      setMatchingResponse (
        const int matchingResponse )
    {
      matchingResponse_ = matchingResponse;
    }
    
  protected: // data
  
    int index_; // is it really a index? if yes, index for what?
    int clusterIndex_;
    int classIndex_;
    
    int maxResponse_;
    
    /**
     * \brief The matching response corresponding to this DOT candidate.
     */
    int matchingResponse_;
    
    int row_;
    int col_;
    
    CvRect roi_;
    
  };
  
  
  void 
    emptyPointerList( 
      std::list< ::ftd::Candidate* > & list );


  /**
   * \brief Supplies function for comparing two candidates based on their 
   *        machting response.
   */
  struct MatchingResponseBasedCandidatePointerComparison
  {
  
    /**
     * \brief Compares two candidates based on their matching response.
     */
    bool 
      operator () (
        Candidate * first,
        Candidate * second )
    {
      return first->getMatchingResponse () - second->getMatchingResponse ();
    }
  };  
    
  
  /**
   * \brief Used to store a list of DOT templates.
   */
  class List
  {
    
  public: // functions
  
    // TODO: add comments
  
    List () 
      : index_ (0),
        clusterIndex_ (0),
        classIndex_ (0),
        sta_ (NULL),
        bitList_ (NULL),
        next_ (NULL),
        flw_ (NULL)
    {};
    ~List () {};
    
    unsigned short getIndex () { return index_; }
    unsigned short getClusterIndex () { return clusterIndex_; }
    unsigned short getClassIndex () { return classIndex_; }
    
    unsigned char * getSTA () { return sta_; }
    unsigned char * getBitList () { return bitList_; }
    
    List * getNext () { return next_; }
    List * getFLW () { return flw_; }
    
    int getOcclusionAmount () { return occlusionAmount_; }
    int getMaxResponse () { return maxResponse_; }
    void setMaxResponse (const int maxResponse) { maxResponse_ = maxResponse; }
    
    void setROI (CvRect & roi) { roi_ = roi; }
    CvRect getROI () { return roi_; }
    
    
    void setIndex (
      const unsigned short index )
    {
      index_ = index;
    }
    
    void setClusterIndex (
      const unsigned short clusterIndex )
    {
      clusterIndex_ = clusterIndex;
    }
    
    void setClassIndex (
      const unsigned short classIndex )
    {
      classIndex_ = classIndex;
    }
    
    void setSTA (
      unsigned char * sta ) 
    {
      sta_ = sta;
    }
    
    void setBitList (
      unsigned char * bitList )
    {
      bitList_ = bitList;
    }
    
    void setNext (
      List * next )
    {
      next_ = next;
    }
    
    void setFLW (
      List * flw )
    {
      flw_ = flw;
    }
    
    void setOcclusionAmount (
      const int occlusionAmount )
    {
      occlusionAmount_ = occlusionAmount;
    }


  public: // data
    
    std::vector<unsigned char*> pyramid;

  protected: // data
  
    unsigned short index_; // is it really a index? if yes, index for what? templateIndex+1
    unsigned short clusterIndex_;
    unsigned short classIndex_;
    
    unsigned char * sta_; // TODO: what means sta???
    unsigned char * bitList_;
    
    int occlusionAmount_;
    
    int maxResponse_;
    
    CvRect roi_;
    
    List * next_;
    List * flw_; // TODO: what means flw???
  };
  
}

#endif 

