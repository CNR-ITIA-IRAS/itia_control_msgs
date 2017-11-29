#ifndef __ITIA_CONTROL_MSGS__
#define __ITIA_CONTROL_MSGS__

# include <itia_control_msgs/ImpedanceParameters.h>
# include <ros/ros.h>
namespace itia_control_msgs
{

inline bool getImpPar ( const std::string& string, itia_control_msgs::ImpedanceParameters* par )
{
  bool res;
  par->kp.resize ( 6 );
  par->kd.resize ( 6 );
  par->m.resize ( 6 );

  ros::NodeHandle nh;
  std::vector<double> kp, kd, m;


  res = res && nh.getParam ( string + "/K", kp );
  res = res && nh.getParam ( string + "/xci", kd );
  res = res && nh.getParam ( string + "/mass", m );

  if ( res )
  {
    for ( int idx = 0; idx < 6; idx++ )
    {
      par->kp.at ( idx ).data = kp.at ( idx );
      par->kd.at ( idx ).data = kd.at ( idx );
      par->m.at ( idx ).data  = m.at ( idx );
    }

    ROS_DEBUG_STREAM ( "Parameter " << string << " loaded" );
  }
  else
    ROS_WARN_STREAM ( "Parameter " << string << " not loaded" );

  return res;
}

}

#endif