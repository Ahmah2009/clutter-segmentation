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

#ifndef UTIL_TIMER_H_
#define UTIL_TIMER_H_ UTIL_TIMER_H_

#include <opencv/cv.h>
#include <opencv/highgui.h>


namespace util
{

  class Timer
  {
  
  public:
  
#ifdef WIN32
	  Timer() : m_time(0)
	  {LARGE_INTEGER l_tmp; ::QueryPerformanceFrequency(&l_tmp); m_freq=l_tmp.QuadPart;}

	  void start( bool a_reset=true)
	  {LARGE_INTEGER l_tmp; ::QueryPerformanceCounter(&l_tmp); if (a_reset) m_time=0; m_start=l_tmp.QuadPart;}

	  void stop()
	  {LARGE_INTEGER l_tmp; ::QueryPerformanceCounter(&l_tmp); m_stop=l_tmp.QuadPart; m_time+=m_stop-m_start;}
#else
	  Timer()
	  {m_time.tv_sec = 0; m_time.tv_nsec = 0;}

	  void start( bool a_reset=true)
	  {
	    if (a_reset) {
	      m_time.tv_sec = 0; 
	      m_time.tv_nsec = 0;
	    } 
	    clock_gettime(CLOCK_REALTIME, &m_start);
	  }

	  void stop()
	  {
	    clock_gettime(CLOCK_REALTIME, &m_stop); m_time = diff(m_start, m_stop);
	  }
	
    timespec diff(timespec start, timespec end)
    {
	    timespec temp;
	    if ((end.tv_nsec-start.tv_nsec)<0) {
		    temp.tv_sec = end.tv_sec-start.tv_sec-1;
		    temp.tv_nsec = 1000000000+end.tv_nsec-start.tv_nsec;
	    } else {
		    temp.tv_sec = end.tv_sec-start.tv_sec;
		    temp.tv_nsec = end.tv_nsec-start.tv_nsec;
	    }
	    return temp;
    }	
#endif
	
#ifdef WIN32
	  double getTime()
	  {return(double)m_time/(double)m_freq;};
	
	  double getFPS() 
	  {return(double)m_freq/(double)m_time;};

	  long getClocks()
	  {return m_time;};
#else
	  double getTime()
	  {return (double)m_time.tv_sec + (double)m_time.tv_nsec/1000000000.0;};
	
	  double getFPS() 
	  {return 1.0/getTime();};
#endif

  private:
  
#ifdef WIN32
	  __int64 m_freq;
	  __int64 m_start, m_stop, m_time;
#else
    timespec m_start, m_stop, m_time;
#endif

  };
  
}


#endif // UTIL_TIMER_H_


