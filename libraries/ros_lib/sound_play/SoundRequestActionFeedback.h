#ifndef _ROS_sound_play_SoundRequestActionFeedback_h
#define _ROS_sound_play_SoundRequestActionFeedback_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "actionlib_msgs/GoalStatus.h"
#include "sound_play/SoundRequestFeedback.h"

namespace sound_play
{

  class SoundRequestActionFeedback : public ros::Msg
  {
    public:
      std_msgs::Header header;
      actionlib_msgs::GoalStatus status;
      sound_play::SoundRequestFeedback feedback;

    SoundRequestActionFeedback():
      header(),
      status(),
      feedback()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->status.serialize(outbuffer + offset);
      offset += this->feedback.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->status.deserialize(inbuffer + offset);
      offset += this->feedback.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "sound_play/SoundRequestActionFeedback"; };
    const char * getMD5(){ return "4577e8c5420cc167578de330a5e37d82"; };

  };

}
#endif