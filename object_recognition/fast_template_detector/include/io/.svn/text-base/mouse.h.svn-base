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

#ifndef IO_MOUSE_H_
#define IO_MOUSE_H_ IO_MOUSE_H_

#include <opencv/cv.h>
#include <opencv/highgui.h>


/**
 * \brief Namespace for IO.
 */
namespace io
{

  /**
   * \brief Class for handling mouse inputs.
   */
  class Mouse
  {
  
  public:
  
    /**
     * \brief A pointer to this function is supplied to OpenCV and called when ever an event occurs.
     */
	  static void cvOnMouse( 
	    int event, 
	    int posX, 
	    int posY, 
	    int flags, 
	    void * params )
	  { 
	    event_ = event; 
	    posX_ = posX; 
	    posY_ = posY; 
	  }
	  
	  /**
	   * \brief Starts the mouse handling.
	   */
	  static void start( 
	    std::string imageName )
	  { 
	    cvSetMouseCallback( imageName.c_str(), ::io::Mouse::cvOnMouse, 0 ); 
	  }
	  
	  /**
	   * \brief Returns the last event and resets the internal event buffer.
	   */
	  static int getEvent() 
	  { 
	    int event = event_; 
	    event_ = -1; 
	    return event;
	  }
	  
	  /**
	   * \brief Returns the last x-position and resets the corresponding internal buffer.
	   */
	  static int getX() 
	  { 
	    int posX = posX_; 
	    //posX_ = -1; 
	    return posX;
	  }
	  
	  /**
	   * \brief Returns the last y-position and resets the corresponding internal buffer.
	   */
	  static int getY() 
	  { 
	    int posY = posY_; 
	    //posY_ = -1; 
	    return posY;
	  }

  private:
  
    /** \brief Stores the last mouse event. */
	  static int event_;
	  /** \brief Stores the last x-position of the mouse. */
	  static int posX_;
	  /** \brief Stores the last y-position of the mouse. */
	  static int posY_;
	  
  };
  
}

#endif // IO_MOUSE_H_

