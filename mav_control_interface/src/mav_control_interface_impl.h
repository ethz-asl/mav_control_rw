/*
 * Copyright (c) 2015, Markus Achtelik, ASL, ETH Zurich, Switzerland
 * You can contact the author at <markus dot achtelik at mavt dot ethz dot ch>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef MAV_CONTROL_INTERFACE_IMPL_H
#define MAV_CONTROL_INTERFACE_IMPL_H

#include <deque>

#include <ros/ros.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <nav_msgs/Odometry.h>
#include <std_srvs/Empty.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include <mav_control_interface/deadzone.h>
#include <mav_control_interface/position_controller_interface.h>
#include <mav_control_interface/rc_interface.h>

#include "state_machine.h"

namespace mav_control_interface {

class MavControlInterfaceImpl
{
 public:
  MavControlInterfaceImpl(ros::NodeHandle& nh, ros::NodeHandle& private_nh,
                          std::shared_ptr<PositionControllerInterface> controller,
                          std::shared_ptr<RcInterfaceBase> rc_interface);

  virtual ~MavControlInterfaceImpl();

 private:
  static constexpr double kOdometryWatchdogTimeout = 1.0;  // seconds

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  ros::Subscriber odometry_subscriber_;
  ros::Subscriber command_trajectory_subscriber_;
  ros::Subscriber command_trajectory_array_subscriber_;
  ros::Timer odometry_watchdog_;

  ros::ServiceServer takeoff_server_;
  ros::ServiceServer back_to_position_hold_server_;

  std::shared_ptr<RcInterfaceBase> rc_interface_;

  std::unique_ptr<state_machine::StateMachine> state_machine_;

  void CommandPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg);
  void CommandTrajectoryCallback(const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& msg);
  void OdometryCallback(const nav_msgs::OdometryConstPtr& msg);
  void OdometryWatchdogCallback(const ros::TimerEvent& e);
  void RcUpdatedCallback(const RcInterfaceBase&);
  bool TakeoffCallback(std_srvs::EmptyRequest& request, std_srvs::EmptyResponse& response);
  bool BackToPositionHoldCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

  void publishAttitudeCommand(const mav_msgs::RollPitchYawrateThrust& command);
};

} /* namespace mav_control_interface */

#endif /* LOW_LEVEL_FLIGHT_MANAGER_H_ */
