#ifndef _ROS_turtlebot_arm_block_manipulation_PickAndPlaceFeedback_h
#define _ROS_turtlebot_arm_block_manipulation_PickAndPlaceFeedback_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace turtlebot_arm_block_manipulation
{

  class PickAndPlaceFeedback : public ros::Msg
  {
    public:

    PickAndPlaceFeedback()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
     return offset;
    }

    const char * getType(){ return "turtlebot_arm_block_manipulation/PickAndPlaceFeedback"; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

}
#endif