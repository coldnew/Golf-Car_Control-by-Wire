#ifndef _ROS_SERVICE_SetComplianceMargin_h
#define _ROS_SERVICE_SetComplianceMargin_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace dynamixel_controllers
{

static const char SETCOMPLIANCEMARGIN[] = "dynamixel_controllers/SetComplianceMargin";

  class SetComplianceMarginRequest : public ros::Msg
  {
    public:
      uint8_t margin;

    SetComplianceMarginRequest():
      margin(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->margin >> (8 * 0)) & 0xFF;
      offset += sizeof(this->margin);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->margin =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->margin);
     return offset;
    }

    const char * getType(){ return SETCOMPLIANCEMARGIN; };
    const char * getMD5(){ return "daacbf1c0642fe923f2dfb9217a97b81"; };

  };

  class SetComplianceMarginResponse : public ros::Msg
  {
    public:

    SetComplianceMarginResponse()
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

    const char * getType(){ return SETCOMPLIANCEMARGIN; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class SetComplianceMargin {
    public:
    typedef SetComplianceMarginRequest Request;
    typedef SetComplianceMarginResponse Response;
  };

}
#endif
