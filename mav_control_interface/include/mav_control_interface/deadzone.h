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

#ifndef DEADZONE_H_
#define DEADZONE_H_

#include <type_traits>

namespace mav_control_interface {

/**
 * \brief This class implements a deadzone for normalized inputs (float or double) from -1.0 ... 1.0
 * \param[in] deadzone Desired deadzone into one direction, between 0.0 ... 1.0. E.g. setting it to 0.1 for
 *                     an RC means results in a deadzone of +-10% of the stick around the center.
 *
 * Inputs larger than |1| will be limited to |1|. This class does not simply cut out the region within the
 * deadzone, but maps the values outside the deadzone to outputs starting from 0, as shown below:
 * \verbatim
 *
 *             |        /
 *             |       /
 *             |      /
 *        ___________
 *       /     |
 *      /      |
 *     /       |
 *
 * \endverbatim
 */
template<typename T>
class Deadzone
{
  static_assert(std::is_same<T, double>::value || std::is_same<T, float>::value,
      "Use this for float or double only");
 public:
  Deadzone(T deadzone)
      : deadzone_(deadzone), slope_(1.0), offset_(0.0)
  {
    if (deadzone_ < 1.0 && deadzone_ >= 0.0) {
      slope_ = 1.0 / (1.0 - deadzone_);
      offset_ = -slope_ * deadzone_;
    }
    // Let's not do anything otherwise.
    else {
      deadzone_ = 1.0;
      slope_ = 0.0;
      offset_ = 0.0;
    }
  }

  T operator()(T input) const
  {
    if (input >= deadzone_) {
      if (input > 1.0) {
        input = 1.0;
      }
      return slope_ * input + offset_;
    }
    else if (input <= -deadzone_) {
      if (input < -1.0) {
        input = -1.0;
      }
      return slope_ * input - offset_;
    }
    return 0.0;
  }

 private:
  T deadzone_; ///< Deadzone size. The deadzone is +- deadzone around 0.
  T slope_; ///< Slope of the new input-to output mapping.
  T offset_; ///< Offset, such that the output is zero when input=deadzone.
};

}



#endif /* DEADZONE_H_ */
