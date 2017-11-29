#ifndef __ITIA_CONTROL_MSGS_MOTION_STAMPED__
#define __ITIA_CONTROL_MSGS_MOTION_STAMPED__
#include <kdl_conversions/kdl_msg.h>
#include <itia_control_msgs/MotionStamped.h>
#include <kdl/frameacc.hpp>

namespace itia_control_msgs
{
  KDL::FrameAcc motionStampedToFrameAcc( itia_control_msgs::MotionStamped motion_msg );  
}

#endif
