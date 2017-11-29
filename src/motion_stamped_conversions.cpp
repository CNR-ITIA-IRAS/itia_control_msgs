
 // -------------------------------------------------------------------------------- 
 // Copyright (c) 2017 CNR-ITIA <iras@itia.cnr.it>
 // All rights reserved.
 //
 // Redistribution and use in source and binary forms, with or without
 // modification, are permitted provided that the following conditions are met:
 //
 // 1. Redistributions of source code must retain the above copyright notice,
 // this list of conditions and the following disclaimer.
 // 2. Redistributions in binary form must reproduce the above copyright
 // notice, this list of conditions and the following disclaimer in the
 // documentation and/or other materials provided with the distribution.
 // 3. Neither the name of mosquitto nor the names of its
 // contributors may be used to endorse or promote products derived from
 // this software without specific prior written permission.
 //
 //
 // THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 // AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 // IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 // ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 // LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 // CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 // SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 // INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 // CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 // ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 // POSSIBILITY OF SUCH DAMAGE.
 // -------------------------------------------------------------------------------- 

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
