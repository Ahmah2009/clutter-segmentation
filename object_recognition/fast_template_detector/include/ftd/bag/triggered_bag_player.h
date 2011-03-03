//////////////////////////////////////////////////////////////////////////////
// 
// Authors: Stefan Holzer 2010 
// Version: 1.0 
//  
//////////////////////////////////////////////////////////////////////////////

#ifndef FTD_BAG_TRIGGERED_BAG_PLAYER_H_
#define FTD_BAG_TRIGGERED_BAG_PLAYER_H_ FTD_BAG_TRIGGERED_BAG_PLAYER_H_

#include <util/util.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <emmintrin.h>

#include <fstream>
#include <iostream>
#include <vector>
#include <list>

#include <time.h>
#include <sys/stat.h>
#include "ros/ros.h"
#include "ros/time.h"
#include <string>

#include "rosrecord/AnyMsg.h"
#include "rosrecord/time_publisher.h"

#include <unistd.h>
#include <termios.h>

// We know this is deprecated..
#define IGNORE_ROSRECORD_DEPRECATED
#undef ROSCPP_DEPRECATED
#define ROSCPP_DEPRECATED
#include "rosrecord/Player.h"

#include "rosrecord/Player.h"
#include "rosrecord/AnyMsg.h"


/**
 * \namespace Namespace ftd for fast template detection.
 * \brief Namespace for fast template detection.
 */
namespace ftd
{

  /**
   * \namespace Namespace for bag-file related stuff.
   * \brief Namespace for bag-file related stuff.
   */
  namespace bag
  {
 
    class TriggeredBagPlayer
    {
        
    public: // functions
      
      TriggeredBagPlayer (
        std::string & bagFileName,
        std::vector<std::string> & topics);
      virtual ~TriggeredBagPlayer ();  
      
      /** \brief Starts the player. */
      bool 
        start ();
        
      /** \brief Triggers the player. */
      bool 
        trigger ();
        
        
    protected: // functions

      /** \brief Generic message handler which receives all messages in the bag file. */
      void    
        messageHandler(
          std::string name,         // Topic name
          ros::Message* m,          // Message pointer
          ros::Time t,              // Shifted and scaled time
          ros::Time t_orig,         // Message timestamp
          void* n);
        
    
    private: // data

      // the player    
      ros::record::Player player_;

      // messages
      std::vector<std::string> handledMessageNames_;
      // counters for messages
      std::vector<int> handledMessageCounters_;
      // current message index
      int currentMessageIndex_;
      // flag that indicates whether we have to wait for a trigger
      bool waitForTrigger_;    
      
      // publishers
      std::map<std::string, ros::Publisher> publishers_;
      SimpleTimePublisher* bag_time_publisher_;
      
      double time_scale_;
      int queue_size_;
      ros::NodeHandle* node_handle;
      ros::WallDuration advertise_sleep_;
      bool at_once_;
      
      ros::WallTime start_time_, requested_start_time_, paused_time_;
      
    };
  
  }
}

#endif // FTD_BAG_TRIGGERED_BAG_PLAYER_H_

