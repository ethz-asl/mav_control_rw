/*
* Copyright (c) 2017, Zachary Taylor, ASL, ETH Zurich, Switzerland
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

#include <mav_control_interface/rc_interface_mavros.h>

namespace mav_control_interface {

RcInterfaceMavRos::RcInterfaceMavRos(const ros::NodeHandle& nh)
    : RcInterfaceBase(),
      nh_(nh),
      is_on_(false)
{
  #ifdef USING_MAVROS
    rc_sub_ = nh_.subscribe("rc", 1, &RcInterfaceMavRos::rcCallback, this);
  #else
    ROS_FATAL("Interface was compiled without Mavros, DO NOT FLY!");
  #endif
}

#ifdef USING_MAVROS
  void RcInterfaceMavRos::rcCallback(const mavros_msgs::RCInPtr& msg)
  {
    is_on_ = isRcOn(msg);
    last_data_.timestamp = msg->header.stamp;

    std::vector<float> rc_data;
    for(uint16_t i : msg->channels){
      //change to range of -1 to 1
      float norm_value = (static_cast<float>(i)-STICK_MIN) / (STICK_MAX-STICK_MIN);
      norm_value = 2*norm_value - 1;

      rc_data.push_back(norm_value);
    }

    if (is_on_) {
      last_data_.right_up_down = rc_data[0];
      last_data_.right_side = -rc_data[0];
      last_data_.left_up_down = rc_data[0];
      last_data_.left_side = -rc_data[0];

      if (rc_data[6] > 0.5)
        last_data_.control_interface = RcData::ControlInterface::ON;
      else
        last_data_.control_interface = RcData::ControlInterface::OFF;

      if (rc_data[4] <= -0.5)
        last_data_.control_mode = RcData::ControlMode::MANUAL;
      else if (rc_data[4] > -0.5 && rc_data[4] < 0.5)
        last_data_.control_mode = RcData::ControlMode::ALTITUDE_CONTROL;
      else
        last_data_.control_mode = RcData::ControlMode::POSITION_CONTROL;

      last_data_.wheel = 0.0;
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
#endif

std::string RcInterfaceMavRos::getName() const
{
  return std::string("MavRos rc interface");
}

RcData RcInterfaceMavRos::getRcData() const
{
  return last_data_;
}

bool RcInterfaceMavRos::isActive() const
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

bool RcInterfaceMavRos::isOn() const
{
  return is_on_;
}

float RcInterfaceMavRos::getStickDeadzone() const
{
  return STICK_DEADZONE;
}

#ifdef USING_MAVROS
  bool RcInterfaceMavRos::isRcOn(const mavros_msgs::RCInPtr& msg) const
  {
    return (msg->rssi != 0);
  }
#endif

}  // end namespace mav_control_interface
