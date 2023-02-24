// Copyright (c) 2016 The UUV Simulator Authors.
// All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef __THRUSTER_ROS_PLUGIN_HH__
#define __THRUSTER_ROS_PLUGIN_HH__

#include <map>
#include <string>
#include <vector>

#include <calypso_thruster/ThrusterPlugin.hh>

#include <boost/scoped_ptr.hpp>
#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include <calypso_thruster/FloatStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>

#include <calypso_thruster/SetThrusterState.h>
#include <calypso_thruster/GetThrusterState.h>
#include <calypso_thruster/SetThrusterEfficiency.h>
#include <calypso_thruster/GetThrusterEfficiency.h>
#include <calypso_thruster/GetThrusterConversionFcn.h>

namespace uuv_simulator_ros
{
  class ThrusterROSPlugin : public gazebo::ThrusterPlugin
  {
    /// \brief Constrcutor.
    public: ThrusterROSPlugin();

    /// \brief Destructor.
    public: ~ThrusterROSPlugin();

    /// \brief Load module and read parameters from SDF.
    public: void Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    /// \brief Publish thruster state via ROS.
    public: void RosPublishStates();

    /// \brief Set new set point (desired thrust [N]) for thruster.
    public: void SetThrustReference(
        const calypso_thruster::FloatStamped::ConstPtr &_msg);

    /// \brief Return the ROS publish period.
    public: gazebo::common::Time  GetRosPublishPeriod();

    /// \brief Set the ROS publish frequency (Hz).
    public: void SetRosPublishRate(double _hz);

    /// \brief Initialize Module.
    public: virtual void Init();

    /// \brief Reset Module.
    public: virtual void Reset();

    /// \brief Set the thrust efficiency factor
    public: bool SetThrustForceEfficiency(
      calypso_thruster::SetThrusterEfficiency::Request& _req,
      calypso_thruster::SetThrusterEfficiency::Response& _res);

    /// \brief Get the thrust efficiency factor
    public: bool GetThrustForceEfficiency(
      calypso_thruster::GetThrusterEfficiency::Request& _req,
      calypso_thruster::GetThrusterEfficiency::Response& _res);

    /// \brief Set the dynamic state efficiency factor
    public: bool SetDynamicStateEfficiency(
      calypso_thruster::SetThrusterEfficiency::Request& _req,
      calypso_thruster::SetThrusterEfficiency::Response& _res);

      /// \brief Get the dynamic state efficiency factor
    public: bool GetDynamicStateEfficiency(
        calypso_thruster::GetThrusterEfficiency::Request& _req,
        calypso_thruster::GetThrusterEfficiency::Response& _res);

    /// \brief Turn thruster on/off
    public: bool SetThrusterState(
      calypso_thruster::SetThrusterState::Request& _req,
      calypso_thruster::SetThrusterState::Response& _res);

    /// \brief Get thruster state
    public: bool GetThrusterState(
      calypso_thruster::GetThrusterState::Request& _req,
      calypso_thruster::GetThrusterState::Response& _res);

    /// \brief Get thruster conversion function parameters
    public: bool GetThrusterConversionFcn(
      calypso_thruster::GetThrusterConversionFcn::Request& _req,
      calypso_thruster::GetThrusterConversionFcn::Response& _res);

    /// \brief Map of thruster services
    private: std::map<std::string, ros::ServiceServer> services;

    /// \brief Pointer to this ROS node's handle.
    private: boost::scoped_ptr<ros::NodeHandle> rosNode;

    /// \brief Subscriber reacting to new reference thrust set points.
    private: ros::Subscriber subThrustReference;

    /// \brief Publisher for current actual thrust.
    private: ros::Publisher pubThrust;

    /// \brief Publisher for current actual thrust as wrench.
    private: ros::Publisher pubThrustWrench;

    /// \brief Publisher for the thruster state
    private: ros::Publisher pubThrusterState;

    /// \brief Publisher for the thrust force efficiency
    private: ros::Publisher pubThrustForceEff;

    /// \brief Publisher for the dynamic state efficiency
    private: ros::Publisher pubDynamicStateEff;

    /// \brief Connection for callbacks on update world.
    private: gazebo::event::ConnectionPtr rosPublishConnection;

    /// \brief Period after which we should publish a message via ROS.
    private: gazebo::common::Time rosPublishPeriod;

    /// \brief Last time we published a message via ROS.
    private: gazebo::common::Time lastRosPublishTime;
  };
}

#endif  // __THRUSTER_ROS_PLUGIN_HH__
