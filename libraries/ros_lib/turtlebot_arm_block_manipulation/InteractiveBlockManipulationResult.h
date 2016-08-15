#ifndef _ROS_turtlebot_arm_block_manipulation_InteractiveBlockManipulationResult_h
#define _ROS_turtlebot_arm_block_manipulation_InteractiveBlockManipulationResult_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Pose.h"

namespace turtlebot_arm_block_manipulation
{

  class InteractiveBlockManipulationResult : public ros::Msg
  {
    public:
      geometry_msgs::Pose pickup_pose;
      geometry_msgs::Pose place_pose;

    InteractiveBlockManipulationResult():
      pickup_pose(),
      place_pose()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->pickup_pose.serialize(outbuffer + offset);
      offset += this->place_pose.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->pickup_pose.deserialize(inbuffer + offset);
      offset += this->place_pose.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "turtlebot_arm_block_manipulation/InteractiveBlockManipulationResult"; };
    const char * getMD5(){ return "3fec3f60e60c18ca7b67a7513b211e95"; };

  };

}
#endif