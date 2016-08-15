#ifndef _ROS_SERVICE_StopController_h
#define _ROS_SERVICE_StopController_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace dynamixel_controllers
{

static const char STOPCONTROLLER[] = "dynamixel_controllers/StopController";

  class StopControllerRequest : public ros::Msg
  {
    public:
      const char* controller_name;

    StopControllerRequest():
      controller_name("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_controller_name = strlen(this->controller_name);
      memcpy(outbuffer + offset, &length_controller_name, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->controller_name, length_controller_name);
      offset += length_controller_name;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_controller_name;
      memcpy(&length_controller_name, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_controller_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_controller_name-1]=0;
      this->controller_name = (char *)(inbuffer + offset-1);
      offset += length_controller_name;
     return offset;
    }

    const char * getType(){ return STOPCONTROLLER; };
    const char * getMD5(){ return "df2b10f2f876d82269ae3fc1e0538e11"; };

  };

  class StopControllerResponse : public ros::Msg
  {
    public:
      bool success;
      const char* reason;

    StopControllerResponse():
      success(0),
      reason("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.real = this->success;
      *(outbuffer + offset + 0) = (u_success.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->success);
      uint32_t length_reason = strlen(this->reason);
      memcpy(outbuffer + offset, &length_reason, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->reason, length_reason);
      offset += length_reason;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.base = 0;
      u_success.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->success = u_success.real;
      offset += sizeof(this->success);
      uint32_t length_reason;
      memcpy(&length_reason, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_reason; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_reason-1]=0;
      this->reason = (char *)(inbuffer + offset-1);
      offset += length_reason;
     return offset;
    }

    const char * getType(){ return STOPCONTROLLER; };
    const char * getMD5(){ return "a4d50a34d34f18de48e2436ff1472594"; };

  };

  class StopController {
    public:
    typedef StopControllerRequest Request;
    typedef StopControllerResponse Response;
  };

}
#endif
