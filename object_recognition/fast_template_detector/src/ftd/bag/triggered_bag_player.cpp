#include "ftd/bag/triggered_bag_player.h"

#include "sys/select.h"

#include "rosrecord/AnyMsg.h"
#include "ftd/bag/time_publisher.h"

#include "rosrecord/Player.h"

#include "ros/ros.h"
#include "ros/time.h"

#include <time.h>
#include <sys/stat.h>

#include <string>

#include <unistd.h>
#include <termios.h>

#include "std_msgs/String.h"

#include "rosrecord/AnyMsg.h"
#include "rosrecord/MsgFunctor.h"
#include "rosrecord/constants.h"

#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "rosbag/time_translator.h"

#include "topic_tools/shape_shifter.h"


// ----------------------------------------------------------------------------
ftd::bag::
TriggeredBagPlayer::
TriggeredBagPlayer (
  std::string & bagFileName,
  std::vector<std::string> & topics )
{
  currentMessageIndex_ = 1;
  waitForTrigger_ = false;
  handledMessageNames_.clear ();
  handledMessageCounters_.clear ();
  for (unsigned int messageIndex = 0; messageIndex < topics.size (); ++messageIndex)
  {
    handledMessageNames_.push_back (std::string (topics[messageIndex]));
    handledMessageCounters_.push_back (0);
  }
  
  advertise_sleep_ = ros::WallDuration(.2);
  at_once_ = false;
  
  bag_time_publisher_ = new SimpleTimePublisher(-1.0, time_scale_);
      
  start_time_ = ros::WallTime::now();
  requested_start_time_ = start_time_;
    
  //if (player_.open (bagFileName, ros::Time (0)))
  if (player_.open (bagFileName, ros::Time(start_time_.sec, start_time_.nsec) + ros::Duration().fromSec(-0.0 / 1.0)))
  //if (player_.open (bagFileName, ros::Time(start_time_.sec, start_time_.nsec) + ros::Duration().fromSec(-0.0 / 1.0), 1.0, false))
  {
    player_.addHandler<AnyMsg, TriggeredBagPlayer>(
      std::string("*"), // Topic name
      &TriggeredBagPlayer::messageHandler,     // Handler
      this,
      NULL,             // void*
      false);           // Don't deserialize
    
    node_handle = new ros::NodeHandle;
  
    time_scale_ = 1.0;
    queue_size_ = 100;

    bag_time_publisher_->setTime(player_.getFirstTime());
  }
}


// ----------------------------------------------------------------------------
ftd::bag::
TriggeredBagPlayer::
~TriggeredBagPlayer ()
{
}


// ----------------------------------------------------------------------------
bool
ftd::bag::
TriggeredBagPlayer::
start ()
{
  ++currentMessageIndex_;
  waitForTrigger_ = false;
  while (player_.nextMsg ())  
  {
    ROS_INFO ("MESSAGE PROCESSED");
    if (waitForTrigger_)
    {
      break;
    }
  }
  
  return false;
}


// ----------------------------------------------------------------------------
bool
ftd::bag::
TriggeredBagPlayer::
trigger ()
{
  ++currentMessageIndex_;
  waitForTrigger_ = false;
  while (player_.nextMsg ())  
  {
    if (waitForTrigger_)
    {
      break;
    }
  }
  
  return false;
}



// ----------------------------------------------------------------------------
void 
ftd::bag::
TriggeredBagPlayer::
messageHandler(
  std::string topic_name,         // Topic name
  ros::Message* m,          // Message pointer
  ros::Time _play_time,     // Shifted and scaled time
  ros::Time record_time,    // Message timestamp
  void* n)                  // Void pointer     
{
  // publish message
  
  ros::WallTime play_time(_play_time.sec, _play_time.nsec);

  if (play_time < requested_start_time_)
  {
    bag_time_publisher_->setTime(record_time);
    return;
  }
    
  // We pull latching and callerid info out of the connection_header if it's available (which it always should be)
  bool latching = false;
  std::string callerid("");
  
  /*if (m->__connection_header != NULL)
  {
    M_string::iterator latch_iter = m->__connection_header->find(std::string("latching"));
    if (latch_iter != m->__connection_header->end())
    {
      if (latch_iter->second != std::string("0"))
      {
        latching = true;
      }
    }

    M_string::iterator callerid_iter = m->__connection_header->find(std::string("callerid"));
    if (callerid_iter != m->__connection_header->end())
    {
      callerid = callerid_iter->second;
    }
  }*/

  // We make a unique id composed of the callerid and the topicname allowing us to have separate advertisers
  // for separate latching topics.

  std::string name = callerid + topic_name;
      
  std::map<std::string, ros::Publisher>::iterator pub_token = publishers_.find(name);

  bag_time_publisher_->setWCHorizon(play_time);
  bag_time_publisher_->setHorizon(record_time);

  // advertise the topic to publish
  if (pub_token == publishers_.end())
  {
  
    ROS_INFO ("create publisher");
    ::ros::AdvertiseOptions opts(topic_name, queue_size_, m->__getMD5Sum(), m->__getDataType(), m->__getMessageDefinition());
    // Have to set the latching field explicitly
    //opts.latch = latching;
    ros::Publisher pub = node_handle->advertise(opts);
    publishers_.insert(publishers_.begin(), std::pair<std::string, ros::Publisher>(name, pub));
    pub_token = publishers_.find(name);

    //    ROS_INFO("Sleeping %.3f seconds after advertising %s...",
    //             advertise_sleep_.toSec(), topic_name.c_str());

    bag_time_publisher_->runStalledClock(advertise_sleep_);

    //    ROS_INFO("Done sleeping.\n");

    player_.shiftTime(ros::Duration(advertise_sleep_.sec, advertise_sleep_.nsec));
    play_time += advertise_sleep_;
    bag_time_publisher_->setWCHorizon(play_time);
  }
  
  if (!at_once_)
  {
    while ( /* *paused_ ||*/ !bag_time_publisher_->horizonReached() && node_handle->ok())
    {
      /*  bool charsleftorpaused = true;

      while (charsleftorpaused && node_handle->ok()){
        //Read from stdin:
        
        char c = EOF;

#ifdef __APPLE__

        fd_set testfd;
        FD_COPY(&stdin_fdset_, &testfd);

#else

        fd_set testfd = stdin_fdset_;

#endif

        timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 0;

        if (select(maxfd_, &testfd, NULL, NULL, &tv) > 0)
          c = getc(stdin);

        switch (c){
        case ' ':
          paused_ = !paused_;
          if (paused_) {
            paused_time_ = ros::WallTime::now();
            std::cout << std::endl << "Hit space to resume, or 's' to step.";
            std::cout.flush();
          } else {

            ros::WallDuration shift = ros::WallTime::now() - paused_time_;
            paused_time_ = ros::WallTime::now();
            player_.shiftTime(ros::Duration(shift.sec, shift.nsec));
            play_time += shift;
            bag_time_publisher_->setWCHorizon(play_time);

            std::cout << std::endl << "Hit space to pause.";
            std::cout.flush();
          }
          break;
        case 's':
          if (paused_){

            bag_time_publisher_->stepClock();
            ros::WallDuration shift = ros::WallTime::now() - play_time ;
            paused_time_ = ros::WallTime::now();
            player_.shiftTime(ros::Duration(shift.sec, shift.nsec));
            play_time += shift;
            bag_time_publisher_->setWCHorizon(play_time);

            (pub_token->second).publish(*m);
            return;
          }
          break;
        case EOF:
          if (paused_)
            bag_time_publisher_->runStalledClock(ros::WallDuration(.01));
          else
            charsleftorpaused = false;
        }
      }*/
      
      bag_time_publisher_->runClock(ros::WallDuration(.01));
    }
  } 
  else 
  {
    bag_time_publisher_->stepClock();
  }

  (pub_token->second).publish(*m);
  ROS_INFO ("publish: %s", pub_token->first.c_str ());
  
  

  // check whether the received message is one of the messages we are interested in
  for (unsigned int messageIndex = 0; messageIndex < handledMessageNames_.size (); ++messageIndex)
  {
    if (name.compare (handledMessageNames_[messageIndex]) == 0)
    {
      ROS_INFO ("%s", name.c_str ());
      ++handledMessageCounters_[messageIndex];
    }
  }
  

  // check whether we received sufficient messages for each of the handled messages  
  bool allNecessaryMessagesPublished = true;
  for (unsigned int messageIndex = 0; messageIndex < handledMessageNames_.size (); ++messageIndex)
  {
    if (handledMessageCounters_[messageIndex] < currentMessageIndex_)
    {
      allNecessaryMessagesPublished = false;
      break;
    }
  }
  
  
  // if enough messages have been received then change state to wait for next trigger
  if (allNecessaryMessagesPublished)
  {
    ROS_INFO ("WAIT FOR NEXT TRIGGER");
    ++currentMessageIndex_;
    waitForTrigger_ = true;
  }
  
  usleep(5 * 1000);
}

