#include <itia_control_msgs/motion_stamped_conversions.h>

KDL::FrameAcc motionStampedToFrameAcc( itia_control_msgs::MotionStamped motion_msg )
{
  int order=motion_msg.order.data;

  KDL::FrameAcc frame_acc;
  KDL::Frame kdl_pose;
  tf::poseMsgToKDL(motion_msg.pose,kdl_pose);

  frame_acc.M.R=kdl_pose.M;
  frame_acc.p.p=kdl_pose.p;

  if (order>0)
  {
    KDL::Vector kdl_vec;
    tf::vectorMsgToKDL(motion_msg.lin.at(0),kdl_vec);
    frame_acc.p.v=kdl_vec;
    tf::vectorMsgToKDL(motion_msg.rot.at(0),kdl_vec);
    frame_acc.M.w=kdl_vec;
  }
  
  if (order>1)
  {
    KDL::Vector kdl_vec;
    tf::vectorMsgToKDL(motion_msg.lin.at(1),kdl_vec);
    frame_acc.p.dv=kdl_vec;
    tf::vectorMsgToKDL(motion_msg.rot.at(1),kdl_vec);
    frame_acc.M.dw=kdl_vec;
  }
};
