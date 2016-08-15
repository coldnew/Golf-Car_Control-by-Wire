#ifndef _ROS_turtlebot_arm_block_manipulation_InteractiveBlockManipulationActionResult_h
#define _ROS_turtlebot_arm_block_manipulation_InteractiveBlockManipulationActionResult_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "actionlib_msgs/GoalStatus.h"
#include "turtlebot_arm_block_manipulation/InteractiveBlockManipulationResult.h"

namespace turtlebot_arm_block_manipulation
{

  class InteractiveBlockManipulationActionResult : public ros::Msg
  {
    public:
      std_msgs::Header header;
      actionlib_msgs::GoalStatus status;
      turtlebot_arm_block_manipulation::InteractiveBlockManipulationResult result;

    InteractiveBlockManipulationActionResult():
      header(),
      status(),
      result()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->status.serialize(outbuffer + offset);
      offset += this->result.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->status.deserialize(inbuffer + offset);
      offset += this->result.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "turtlebot_arm_block_manipulation/InteractiveBlockManipulationActionResult"; };
    const char * getMD5(){ return "3d3e4da50890d607335059cdebae3e1d"; };

  };

}
#endif