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

#ifndef RC_INTERFACE_H_
#define RC_INTERFACE_H_

#include <functional>

#include <ros/ros.h>
#include <ros/subscriber.h>
#include <sensor_msgs/Joy.h>

namespace mav_control_interface {

class RcData
{
 public:
  enum class ControlInterface
  {
    ON,
    OFF
  } control_interface;

  enum class ControlMode
  {
    POSITION_CONTROL,
    ALTITUDE_CONTROL,
    MANUAL
  } control_mode;

  RcData()
      : left_up_down(-1.0),
        left_side(0),
        right_up_down(0),
        right_side(0),
        control_interface(ControlInterface::OFF),
        control_mode(ControlMode::MANUAL),
        wheel(0)
  {

  }
  ros::Time timestamp;
  float left_up_down;
  float left_side;
  float right_up_down;
  float right_side;
  float wheel;
};

class RcInterfaceBase {
 public:
  virtual ~RcInterfaceBase() {
  }

  virtual std::string getName() const = 0;
  virtual bool isActive() const = 0;
  virtual bool isOn() const = 0;
  virtual RcData getRcData() const = 0;
  virtual float getStickDeadzone() const = 0;

  template<class T>
  void registerUpdatedCallback(void (T::*method)(const RcInterfaceBase&), T* object)
  {
    updated_callback_ = std::bind(method, object, std::placeholders::_1);
  }
  void unregisterUpdatedCallback();

 protected:
  void updated();

 private:
  std::function<void (const RcInterfaceBase&)> updated_callback_;
};

}  // end namespace mav_control_interface
#endif /* RC_INTERFACE_H_ */
