#ifndef _ROS_turtlebot_arm_block_manipulation_InteractiveBlockManipulationAction_h
#define _ROS_turtlebot_arm_block_manipulation_InteractiveBlockManipulationAction_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "turtlebot_arm_block_manipulation/InteractiveBlockManipulationActionGoal.h"
#include "turtlebot_arm_block_manipulation/InteractiveBlockManipulationActionResult.h"
#include "turtlebot_arm_block_manipulation/InteractiveBlockManipulationActionFeedback.h"

namespace turtlebot_arm_block_manipulation
{

  class InteractiveBlockManipulationAction : public ros::Msg
  {
    public:
      turtlebot_arm_block_manipulation::InteractiveBlockManipulationActionGoal action_goal;
      turtlebot_arm_block_manipulation::InteractiveBlockManipulationActionResult action_result;
      turtlebot_arm_block_manipulation::InteractiveBlockManipulationActionFeedback action_feedback;

    InteractiveBlockManipulationAction():
      action_goal(),
      action_result(),
      action_feedback()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->action_goal.serialize(outbuffer + offset);
      offset += this->action_result.serialize(outbuffer + offset);
      offset += this->action_feedback.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->action_goal.deserialize(inbuffer + offset);
      offset += this->action_result.deserialize(inbuffer + offset);
      offset += this->action_feedback.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "turtlebot_arm_block_manipulation/InteractiveBlockManipulationAction"; };
    const char * getMD5(){ return "6b355c8305ef3300641af877554e1717"; };

  };

}
#endif