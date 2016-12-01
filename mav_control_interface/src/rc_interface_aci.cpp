/*
* Copyright (c) 2015, Markus Achtelik, ASL, ETH Zurich, Switzerland
* Copyright (c) 2014, Sammy Omari, ASL, ETH Zurich, Switzerland
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

#include <mav_control_interface/rc_interface_aci.h>

namespace mav_control_interface {

RcInterfaceAci::RcInterfaceAci(const ros::NodeHandle& nh)
    : RcInterfaceBase(),
      nh_(nh),
      is_on_(false)
{
  rc_sub_ = nh_.subscribe("rc", 1, &RcInterfaceAci::rcCallback, this);
}

void RcInterfaceAci::rcCallback(const sensor_msgs::JoyConstPtr& msg)
{
  is_on_ = isRcOn(msg);
  last_data_.timestamp = msg->header.stamp;

  if (is_on_) {
    last_data_.right_up_down = msg->axes[0];
    last_data_.right_side = -msg->axes[1];
    last_data_.left_up_down = msg->axes[2];
    last_data_.left_side = -msg->axes[3];

    if (msg->axes[5] > 0.0)
      last_data_.control_interface = RcData::ControlInterface::ON;
    else
      last_data_.control_interface = RcData::ControlInterface::OFF;

    if (msg->axes[4] <= -0.5)
      last_data_.control_mode = RcData::ControlMode::MANUAL;
    else if (msg->axes[4] > -0.5 && msg->axes[4] < 0.5)
      last_data_.control_mode = RcData::ControlMode::ALTITUDE_CONTROL;
    else
      last_data_.control_mode = RcData::ControlMode::POSITION_CONTROL;

    last_data_.wheel = msg->axes[6];
  }
  else {  //set to zero if RC is off
    ROS_WARN_STREAM_THROTTLE(5.0, "Detected RC Off.");
    last_data_.right_up_down = 0.0;
    last_data_.right_side = 0.0;
    last_data_.left_up_down = -1.0;
    last_data_.left_side = 0.0;
    last_data_.control_interface = RcData::ControlInterface::OFF;
    last_data_.control_mode = RcData::ControlMode::MANUAL;
    last_data_.wheel = 0.0;
  }
  this->updated();
}

std::string RcInterfaceAci::getName() const
{
  return std::string("ACI rc interface");
}

RcData RcInterfaceAci::getRcData() const
{
  return last_data_;
}

bool RcInterfaceAci::isActive() const
{
  if (!is_on_)
    return false;
  else if (std::abs(last_data_.right_up_down) > STICK_DEADZONE
      || std::abs(last_data_.right_side) > STICK_DEADZONE
      || std::abs(last_data_.left_up_down) > STICK_DEADZONE
      || std::abs(last_data_.left_side) > STICK_DEADZONE) {
    return true;
  }
  return false;
}

bool RcInterfaceAci::isOn() const
{
  return is_on_;
}

float RcInterfaceAci::getStickDeadzone() const
{
  return STICK_DEADZONE;
}

bool RcInterfaceAci::isRcOn(const sensor_msgs::JoyConstPtr& msg) const
{
  return (msg->buttons[0] == 1);
}

}  // end namespace mav_control_interface
