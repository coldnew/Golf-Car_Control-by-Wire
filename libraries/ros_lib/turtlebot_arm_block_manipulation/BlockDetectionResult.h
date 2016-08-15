#ifndef _ROS_turtlebot_arm_block_manipulation_BlockDetectionResult_h
#define _ROS_turtlebot_arm_block_manipulation_BlockDetectionResult_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/PoseArray.h"

namespace turtlebot_arm_block_manipulation
{

  class BlockDetectionResult : public ros::Msg
  {
    public:
      geometry_msgs::PoseArray blocks;

    BlockDetectionResult():
      blocks()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->blocks.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->blocks.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "turtlebot_arm_block_manipulation/BlockDetectionResult"; };
    const char * getMD5(){ return "fe4272fcc0cf26cf952b16d66c620bd4"; };

  };

}
#endif