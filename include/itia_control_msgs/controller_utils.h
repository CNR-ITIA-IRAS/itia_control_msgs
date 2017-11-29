
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

#ifndef __ITIA_CONTROLLER_UTILS__
#define __ITIA_CONTROLLER_UTILS__

#include <ros/ros.h>
#include <controller_manager_msgs/LoadController.h>
#include <controller_manager_msgs/SwitchController.h>
#include <controller_manager_msgs/UnloadController.h>
#include <itia_control_msgs/ImpedanceParameters.h>

namespace itia 
{
namespace control 
{

  inline bool getImpPar (const ros::NodeHandle nh, const std::string& string, itia_control_msgs::ImpedanceParameters* par)
  {
    bool res = true;
    par->kp.resize ( 6 );
    par->kd.resize ( 6 );
    par->m.resize ( 6 );

    std::vector<double> kp, kd, m;


    res = res && nh.getParam ( string + "/K", kp );
    if (!res)
      ROS_DEBUG_STREAM ( "Parameter " << nh.getNamespace() << "/" << string << "/K not found" );
    
    res = res && nh.getParam ( string + "/xci", kd );
    if (!res)
      ROS_DEBUG_STREAM ( "Parameter " << nh.getNamespace() << "/" << string << "/xci not found" );
      
    res = res && nh.getParam ( string + "/mass", m );
    if (!res)
      ROS_DEBUG_STREAM ( "Parameter " << nh.getNamespace() << "/" << string << "/mass not found" );

    if ( res )
    {
      for ( int idx = 0; idx < 6; idx++ )
      {
        par->kp.at ( idx ).data = kp.at ( idx );
        par->kd.at ( idx ).data = kd.at ( idx );
        par->m.at ( idx ).data  = m.at ( idx );
      }

      ROS_DEBUG_STREAM ( "Parameter " << nh.getNamespace() << "/" << string << " loaded" );
    }
    else
      ROS_WARN_STREAM ( "Parameter " << nh.getNamespace() << "/"   << string << " not loaded" );

    return res;
  }
  
  inline bool getImpPar (const std::string& string, itia_control_msgs::ImpedanceParameters* par)
  {
    ros::NodeHandle nh;
    return getImpPar(nh, string, par);
  }
  
  class ControllerManager{
  protected:
    ros::ServiceClient m_load_control_client;
    ros::ServiceClient m_switch_control_client;
    ros::ServiceClient m_unload_control_client;
    ros::NodeHandle    m_nh;
    std::string        m_active_controller;
  public:
    ControllerManager(const ros::NodeHandle& nh)
    {
      m_nh=nh;
      m_load_control_client   = m_nh.serviceClient<controller_manager_msgs::LoadController>("/controller_manager/load_controller");
      m_switch_control_client = m_nh.serviceClient<controller_manager_msgs::SwitchController>("/controller_manager/switch_controller");
      m_unload_control_client = m_nh.serviceClient<controller_manager_msgs::UnloadController>("/controller_manager/unload_controller");
    }
  
    void loadController(const std::string& ctrlname)
    {
      //---------------------------------------------------------------------------------------------------------------
      controller_manager_msgs::LoadController load_ctrl_msg;
      load_ctrl_msg.request.name = ctrlname;

      if (!m_load_control_client.waitForExistence(ros::Duration(10)))
      {
        throw std::runtime_error("No control managers available");
      }
      else
        ROS_INFO_STREAM("Loading controller: " <<  ctrlname);

      if (!m_load_control_client.call( load_ctrl_msg ))
      {
        throw std::runtime_error( std::string("Error loading controller : " +  ctrlname ).c_str() );
      }
      if (!load_ctrl_msg.response.ok)
      {
        throw std::runtime_error( std::string("Error loading controller: " +  ctrlname ).c_str() );
      }
      ROS_INFO("starting controller");
      controller_manager_msgs::SwitchController switch_ctrl_msg;

      switch_ctrl_msg.request.start_controllers.resize(1);
      switch_ctrl_msg.request.start_controllers.at(0) = ctrlname;
      switch_ctrl_msg.request.strictness = 1;
      if (!m_switch_control_client.call( switch_ctrl_msg ))
      {
        throw std::runtime_error( std::string("Error starting controller: " +  ctrlname ).c_str() );
      }
      if (!switch_ctrl_msg.response.ok)
      {
        throw std::runtime_error( std::string("Error starting controller: " +  ctrlname ).c_str() );
      }
      m_active_controller=ctrlname;
    };

    void unloadController(const std::string& ctrlname) 
    {
      controller_manager_msgs::SwitchController switch_ctrl_msg;

      switch_ctrl_msg.request.start_controllers.resize(1);
      switch_ctrl_msg.request.start_controllers.at(0) = ctrlname;
      switch_ctrl_msg.request.strictness = 1;

      switch_ctrl_msg.request.start_controllers.resize(0);
      switch_ctrl_msg.request.stop_controllers.resize(1);
      switch_ctrl_msg.request.stop_controllers.at(0) = ctrlname;
      switch_ctrl_msg.request.strictness = 1;
      if (!m_switch_control_client.call( switch_ctrl_msg ))
      {
        throw std::runtime_error( std::string("Error stopping controller: " +  ctrlname ).c_str() );
      }
      if (!switch_ctrl_msg.response.ok)
      {
        throw std::runtime_error( std::string("Error stopping controller: " +  ctrlname ).c_str() );
      }
      ROS_INFO_STREAM("stopping controller: " <<  ctrlname);
      controller_manager_msgs::UnloadController unload_ctrl_msg;
      unload_ctrl_msg.request.name = ctrlname;
      if (!m_unload_control_client.call( unload_ctrl_msg ))
      {
        throw std::runtime_error( std::string("Error unloading controller: " +  ctrlname ).c_str() );
      }
      if (!unload_ctrl_msg.response.ok)
      {
        throw std::runtime_error( std::string("Error unloading controller: " +  ctrlname ).c_str() );
      }
      ROS_INFO_STREAM("Unloading controller: " <<  ctrlname);
      m_active_controller="";
    //---------------------------------------------------------------------------------------------------------------
    };
    
    void unloadController()
    {
      
      if (!m_active_controller.compare(""))
      {
        ROS_INFO("No active controllers");
      }
      controller_manager_msgs::SwitchController switch_ctrl_msg;
      
      switch_ctrl_msg.request.start_controllers.resize(1);
      switch_ctrl_msg.request.start_controllers.at(0) = m_active_controller;
      switch_ctrl_msg.request.strictness = 1;

      switch_ctrl_msg.request.start_controllers.resize(0);
      switch_ctrl_msg.request.stop_controllers.resize(1);
      switch_ctrl_msg.request.stop_controllers.at(0) = m_active_controller;
      switch_ctrl_msg.request.strictness = 1;
      if (!m_switch_control_client.call( switch_ctrl_msg ))
      {
        throw std::runtime_error( std::string("Error stopping controller: " +  m_active_controller ).c_str() );
      }
      if (!switch_ctrl_msg.response.ok)
      {
        throw std::runtime_error( std::string("Error stopping controller: " +  m_active_controller ).c_str() );
      }
      ROS_INFO_STREAM("stopping controller: " <<  m_active_controller);
      controller_manager_msgs::UnloadController unload_ctrl_msg;
      unload_ctrl_msg.request.name = m_active_controller;
      if (!m_unload_control_client.call( unload_ctrl_msg ))
      {
        throw std::runtime_error( std::string("Error unloading controller: " +  m_active_controller ).c_str() );
      }
      if (!unload_ctrl_msg.response.ok)
      {
        throw std::runtime_error( std::string("Error unloading controller: " +  m_active_controller ).c_str() );
      }
      ROS_INFO_STREAM("Unloading controller: " <<  m_active_controller);
      m_active_controller="";
    }
    
    void switchController(const std::string& start_ctrlname, const std::string& stop_ctrlname) 
    {
       controller_manager_msgs::SwitchController switch_ctrl_msg;

      if (start_ctrlname.compare(""))
      {
        switch_ctrl_msg.request.start_controllers.resize(1);
        switch_ctrl_msg.request.start_controllers.at(0) = start_ctrlname;
      }
      else
        switch_ctrl_msg.request.start_controllers.resize(0);
      
      if (stop_ctrlname.compare(""))
      {
        switch_ctrl_msg.request.stop_controllers.resize(1);
        switch_ctrl_msg.request.stop_controllers.at(0) = stop_ctrlname;
      }
      else
        switch_ctrl_msg.request.stop_controllers.resize(0);

      switch_ctrl_msg.request.strictness = 2;
      if (!m_switch_control_client.call( switch_ctrl_msg ))
      {
        throw std::runtime_error( std::string("Error switching from controller: " +  stop_ctrlname + " to: " + start_ctrlname).c_str() );
      }
      if (!switch_ctrl_msg.response.ok)
      {
        throw std::runtime_error( std::string("Error switching from controller: " +  stop_ctrlname + " to: " + start_ctrlname).c_str() );
      }
      ROS_DEBUG_STREAM("Switched from controller: " <<  stop_ctrlname << " to: " << start_ctrlname);
      m_active_controller=start_ctrlname;
    }
    
    
  };

}
}


#endif