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

#ifndef MAV_CONTROL_INTERFACE_PARAMETERS_H_
#define MAV_CONTROL_INTERFACE_PARAMETERS_H_

#include <math.h>

#include <mav_control_interface/deadzone.h>

namespace mav_control_interface {

class Parameters {
 public:
  static constexpr double kDefaultStickDeadzone = 0.1;
  static constexpr double kDefaultRcTeleopMaxCarrotDistancePosition = 1.0;
  static constexpr double kDefaultRcTeleopMaxCarrotDistanceYaw = M_PI / 4.0;
  static constexpr double kDefaultRcMaxRollPitchCommand = 45.0 / 180.0 * M_PI;
  static constexpr double kDefaultRcMaxYawRateCommand = 45.0 / 180.0 * M_PI;
  static constexpr double kDefaultTakeoffDistance = 1.0;
  static constexpr double kDefaultTakeoffTime = 5.0;

  Parameters()
      : stick_deadzone_(kDefaultStickDeadzone),
        rc_teleop_max_carrot_distance_position_(kDefaultRcTeleopMaxCarrotDistancePosition),
        rc_teleop_max_carrot_distance_yaw_(kDefaultRcTeleopMaxCarrotDistanceYaw),
        rc_max_roll_pitch_command_(kDefaultRcMaxRollPitchCommand),
        rc_max_yaw_rate_command_(kDefaultRcMaxYawRateCommand),
        takeoff_distance_(kDefaultTakeoffDistance),
        takeoff_time_(kDefaultTakeoffTime)
  {
  }

  Deadzone<double> stick_deadzone_;
  double rc_teleop_max_carrot_distance_position_;
  double rc_teleop_max_carrot_distance_yaw_;
  double rc_max_roll_pitch_command_;
  double rc_max_yaw_rate_command_;
  double takeoff_distance_;
  double takeoff_time_;
};

} // end namespace mav_control_interface

#endif /* MAV_CONTROL_INTERFACE_PARAMETERS_H_ */
