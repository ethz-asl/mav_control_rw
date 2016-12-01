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

#ifndef MAV_CONTROL_INTERFACE_H_
#define MAV_CONTROL_INTERFACE_H_

#include <memory>

#include <mav_control_interface/position_controller_interface.h>
#include <mav_control_interface/rc_interface.h>
#include <ros/ros.h>

namespace mav_control_interface {

class MavControlInterfaceImpl;

class MavControlInterface
{
 public:
  MavControlInterface(ros::NodeHandle& nh, ros::NodeHandle& private_nh,
                      std::shared_ptr<PositionControllerInterface> controller,
                      std::shared_ptr<RcInterfaceBase> rc_interface);

  ~MavControlInterface();
 private:
  std::unique_ptr<MavControlInterfaceImpl> mav_control_interface_impl_;
};

} /* namespace mav_control_interface */

#endif /* MAV_CONTROL_INTERFACE_H_ */
