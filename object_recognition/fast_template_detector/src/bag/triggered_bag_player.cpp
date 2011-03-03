#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <stereo_msgs/DisparityImage.h>
#include <cv_bridge/CvBridge.h>
#include <image_transport/image_transport.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <ftd/fast_template_detector_vs.h>
#include <fast_template_detector/DiscretizedData.h>
#include <fast_template_detector/DetectionCandidates.h>
#include <fast_template_detector/Trigger.h>
#include <io/mouse.h>
#include <util/timer.h>

#include <pcl/io/pcd_io.h>

#include <sstream>
#include <string>


#include <string>
#include "rosrecord/Player.h"
#include "rosrecord/AnyMsg.h"

#include "ftd/bag/triggered_bag_player.h"


::ros::NodeHandle * nodeHandle;
::ftd::bag::TriggeredBagPlayer * player;


/*
// messages
std::vector<std::string> handledMessageNames;
// counters for messages
std::vector<int> handledMessageCounters;
// current message index
int currentMessageIndex;
// flag that indicates whether we have to wait for a trigger
bool waitForTrigger;


// Generic callback to be invoked for any message
void 
  all_handler(
    std::string name,         // Topic name
    ros::Message* m,          // Message pointer
    ros::Time t,              // Shifted and scaled time
    ros::Time t_orig,         // Message timestamp
    void* n)                  // Void pointer     
{
  // do we have to publish the message by our self? or is it done automatically?

  // check whether the received message is one of the messages we are interested in
  for (unsigned int messageIndex = 0; messageIndex < handledMessageNames.size (); ++messageIndex)
  {
    if (name.compare (handledMessageNames[messageIndex]) == 0)
    {
      ROS_INFO ("%s", name.c_str ());
      ++handledMessageCounters[messageIndex];
    }
  }
  

  // check whether we received sufficient messages for each of the handled messages  
  bool allNecessaryMessagesPublished = true;
  for (unsigned int messageIndex = 0; messageIndex < handledMessageNames.size (); ++messageIndex)
  {
    if (handledMessageCounters[messageIndex] < currentMessageIndex)
    {
      allNecessaryMessagesPublished = false;
      break;
    }
  }
  
  
  // if enough messages have been received then change state to wait for next trigger
  if (allNecessaryMessagesPublished)
  {
    ROS_INFO ("WAIT FOR NEXT TRIGGER");
    ++currentMessageIndex;
    waitForTrigger = true;
  }
}
*/


void 
  triggerCallback(
    const ::fast_template_detector::TriggerConstPtr & triggerMsg )       
{
  ROS_INFO ("Received Trigger");
  
  player->trigger ();
}


int
main(
  int argc,
  char ** argv )
{
  using ::ftd::bag::TriggeredBagPlayer;

  ::ros::init (argc, argv, "triggered_bag_player");
  nodeHandle = new ::ros::NodeHandle ("~");

  
  ROS_INFO ("number of messages to be handled: %d", argc-2);
  
  //currentMessageIndex = 0;
  //waitForTrigger = false;
  //handledMessageNames.clear ();
  //handledMessageCounters.clear ();
  std::vector<std::string> handledMessageNames;
  for (int messageIndex = 2; messageIndex < argc; ++messageIndex)
  {
    handledMessageNames.push_back (std::string (argv[messageIndex]));
    //handledMessageCounters.push_back (0);
  }
  
  std::string topic = nodeHandle->resolveName("/trigger");
  ::ros::Subscriber subscriber = nodeHandle->subscribe<fast_template_detector::Trigger>(topic, 1, &triggerCallback);
  
  std::string bagName = std::string (argv[1]);
  player = new TriggeredBagPlayer (bagName, handledMessageNames);
  
  player->start ();
  
    
  /*ros::record::Player player;
  if (player.open (std::string (argv[1]), ros::Time (0)))
  {
    player.addHandler<AnyMsg>(
      std::string("*"), // Topic name
      &all_handler,     // Handler
      NULL,             // void*
      false);           // Don't deserialize
  }

  
  // Spin until we have consumed the bag
  while (player.nextMsg ())  
  {
    if (waitForTrigger)
    {
      while (waitForTrigger)
      {
        // sleep a bit
      }
    }
  }*/
  
  ::ros::spin();
}

