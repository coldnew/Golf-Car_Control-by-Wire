#ifndef _ROS_turtlebot_arm_block_manipulation_BlockDetectionActionResult_h
#define _ROS_turtlebot_arm_block_manipulation_BlockDetectionActionResult_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "actionlib_msgs/GoalStatus.h"
#include "turtlebot_arm_block_manipulation/BlockDetectionResult.h"

namespace turtlebot_arm_block_manipulation
{

  class BlockDetectionActionResult : public ros::Msg
  {
    public:
      std_msgs::Header header;
      actionlib_msgs::GoalStatus status;
      turtlebot_arm_block_manipulation::BlockDetectionResult result;

    BlockDetectionActionResult():
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

    const char * getType(){ return "turtlebot_arm_block_manipulation/BlockDetectionActionResult"; };
    const char * getMD5(){ return "8a8cf0cf1fe541495dc3aa68c42af4c2"; };

  };

}
#endif