#ifndef _ROS_turtlebot_arm_block_manipulation_BlockDetectionGoal_h
#define _ROS_turtlebot_arm_block_manipulation_BlockDetectionGoal_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace turtlebot_arm_block_manipulation
{

  class BlockDetectionGoal : public ros::Msg
  {
    public:
      const char* frame;
      float table_height;
      float block_size;

    BlockDetectionGoal():
      frame(""),
      table_height(0),
      block_size(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_frame = strlen(this->frame);
      memcpy(outbuffer + offset, &length_frame, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->frame, length_frame);
      offset += length_frame;
      union {
        float real;
        uint32_t base;
      } u_table_height;
      u_table_height.real = this->table_height;
      *(outbuffer + offset + 0) = (u_table_height.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_table_height.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_table_height.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_table_height.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->table_height);
      union {
        float real;
        uint32_t base;
      } u_block_size;
      u_block_size.real = this->block_size;
      *(outbuffer + offset + 0) = (u_block_size.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_block_size.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_block_size.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_block_size.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->block_size);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_frame;
      memcpy(&length_frame, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_frame; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_frame-1]=0;
      this->frame = (char *)(inbuffer + offset-1);
      offset += length_frame;
      union {
        float real;
        uint32_t base;
      } u_table_height;
      u_table_height.base = 0;
      u_table_height.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_table_height.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_table_height.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_table_height.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->table_height = u_table_height.real;
      offset += sizeof(this->table_height);
      union {
        float real;
        uint32_t base;
      } u_block_size;
      u_block_size.base = 0;
      u_block_size.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_block_size.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_block_size.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_block_size.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->block_size = u_block_size.real;
      offset += sizeof(this->block_size);
     return offset;
    }

    const char * getType(){ return "turtlebot_arm_block_manipulation/BlockDetectionGoal"; };
    const char * getMD5(){ return "55e74460cfe309311425690ecb457a45"; };

  };

}
#endif