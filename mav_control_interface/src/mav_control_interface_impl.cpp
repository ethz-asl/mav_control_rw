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

#include <Eigen/Geometry>
#include <mav_msgs/default_topics.h>
#include <mav_msgs/AttitudeThrust.h>
#include <mav_msgs/RollPitchYawrateThrust.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include "mav_control_interface_impl.h"
#include "parameters.h"

namespace mav_control_interface {

constexpr double MavControlInterfaceImpl::kOdometryWatchdogTimeout;

MavControlInterfaceImpl::MavControlInterfaceImpl(ros::NodeHandle& nh, ros::NodeHandle& private_nh,
                                                 std::shared_ptr<PositionControllerInterface> controller,
                                                 std::shared_ptr<RcInterfaceBase> rc_interface)
    : nh_(nh),
      private_nh_(private_nh),
      rc_interface_(rc_interface)
{
  ros::NodeHandle interface_nh(private_nh, "control_interface");

  odometry_watchdog_ = nh_.createTimer(ros::Duration(kOdometryWatchdogTimeout),
                                       &MavControlInterfaceImpl::OdometryWatchdogCallback, this, false, true);

  command_trajectory_subscriber_ = nh_.subscribe(mav_msgs::default_topics::COMMAND_POSE, 1,
                                                 &MavControlInterfaceImpl::CommandPoseCallback, this);

  command_trajectory_array_subscriber_ = nh_.subscribe(
      mav_msgs::default_topics::COMMAND_TRAJECTORY, 1,
      &MavControlInterfaceImpl::CommandTrajectoryCallback, this);

  odometry_subscriber_ = nh_.subscribe(mav_msgs::default_topics::ODOMETRY, 1,
                                       &MavControlInterfaceImpl::OdometryCallback, this,
                                       ros::TransportHints().tcpNoDelay());

  rc_interface_->registerUpdatedCallback(&MavControlInterfaceImpl::RcUpdatedCallback, this);

  takeoff_server_ = nh.advertiseService("takeoff", &MavControlInterfaceImpl::TakeoffCallback, this);
  back_to_position_hold_server_ = nh.advertiseService("back_to_position_hold",
                                                      &MavControlInterfaceImpl::BackToPositionHoldCallback,
                                                      this);

  state_machine_.reset(new state_machine::StateMachine(nh_, private_nh_, controller));

  Parameters p;
  interface_nh.param("rc_teleop_max_carrot_distance_position", p.rc_teleop_max_carrot_distance_position_,
                     Parameters::kDefaultRcTeleopMaxCarrotDistancePosition);

  interface_nh.param("rc_teleop_max_carrot_distance_yaw", p.rc_teleop_max_carrot_distance_yaw_,
                     Parameters::kDefaultRcTeleopMaxCarrotDistanceYaw);

  if (p.rc_teleop_max_carrot_distance_yaw_ > M_PI / 2.0) {
    p.rc_teleop_max_carrot_distance_yaw_ = M_PI / 2.0;
    ROS_WARN("rc_teleop_max_carrot_distance_yaw_ was limited to pi/2. This is by far enough.");
  }

  interface_nh.param("rc_max_roll_pitch_command", p.rc_max_roll_pitch_command_,
                     Parameters::kDefaultRcMaxRollPitchCommand);
  interface_nh.param("rc_max_yaw_rate_command", p.rc_max_yaw_rate_command_,
                     Parameters::kDefaultRcMaxYawRateCommand);
  interface_nh.param("takeoff_distance", p.takeoff_distance_, Parameters::kDefaultTakeoffDistance);
  interface_nh.param("takeoff_time", p.takeoff_time_, Parameters::kDefaultTakeoffTime);

  p.stick_deadzone_ = Deadzone<double>(rc_interface->getStickDeadzone());
  state_machine_->SetParameters(p);

  ROS_INFO_STREAM("Created control interface for controller " << controller->getName() <<
                  " and RC " << rc_interface_->getName());

  state_machine_->start();
}

MavControlInterfaceImpl::~MavControlInterfaceImpl()
{
  state_machine_->stop();
}

void MavControlInterfaceImpl::RcUpdatedCallback(const RcInterfaceBase& rc_interface)
{
  state_machine_->process_event(
      state_machine::RcUpdate(rc_interface_->getRcData(), rc_interface_->isActive(), rc_interface_->isOn()));
}

void MavControlInterfaceImpl::CommandPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{

  mav_msgs::EigenTrajectoryPoint reference;
  mav_msgs::eigenTrajectoryPointFromPoseMsg(*msg, &reference);

  mav_msgs::EigenTrajectoryPointDeque references;
  references.push_back(reference);

  state_machine_->process_event(state_machine::ReferenceUpdate(references));
}

void MavControlInterfaceImpl::CommandTrajectoryCallback(
    const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& msg)
{
  int array_size = msg->points.size();
  if (array_size == 0)
    return;

  mav_msgs::EigenTrajectoryPointDeque references;
  mav_msgs::eigenTrajectoryPointDequeFromMsg(*msg, &references);

  state_machine_->process_event(state_machine::ReferenceUpdate(references));
}

void MavControlInterfaceImpl::OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg)
{
  ROS_INFO_ONCE("Control interface got first odometry message.");
  mav_msgs::EigenOdometry odometry;
  mav_msgs::eigenOdometryFromMsg(*odometry_msg, &odometry);
  state_machine_->process_event(state_machine::OdometryUpdate(odometry));
}

void MavControlInterfaceImpl::OdometryWatchdogCallback(const ros::TimerEvent& e)
{
  state_machine_->process_event(state_machine::OdometryWatchdog());
}

bool MavControlInterfaceImpl::TakeoffCallback(std_srvs::Empty::Request& request,
                                              std_srvs::Empty::Response& response)
{
  ROS_INFO("Take off event sent");
  state_machine_->process_event(state_machine::Takeoff());
  return true;
}

bool MavControlInterfaceImpl::BackToPositionHoldCallback(std_srvs::Empty::Request& request,
                                                         std_srvs::Empty::Response& response)
{
  state_machine_->process_event(state_machine::BackToPositionHold());
  return true;
}

}  // end namespace mav_control_interface

