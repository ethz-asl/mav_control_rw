/*
 Copyright (c) 2015, Mina Kamel, ASL, ETH Zurich, Switzerland

 You can contact the author at <mina.kamel@mavt.ethz.ch>

 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright
 notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
 notice, this list of conditions and the following disclaimer in the
 documentation and/or other materials provided with the distribution.
 * Neither the name of ETHZ-ASL nor the
 names of its contributors may be used to endorse or promote products
 derived from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL ETHZ-ASL BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef INCLUDE_MAV_CONTROL_STEADYSTATECALCULATION_H_
#define INCLUDE_MAV_CONTROL_STEADYSTATECALCULATION_H_

#include <Eigen/Core>
#include <Eigen/Dense>
#include <iostream>
#include <ros/ros.h>

namespace mav_control {

class SteadyStateCalculation
{
 private:
  static constexpr int kStateSize = 8;
  static constexpr int kInputSize = 3;
  static constexpr int kMeasurementSize = 6;
  static constexpr int kDisturbanceSize = 3;

 public:
  SteadyStateCalculation(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh);
  ~SteadyStateCalculation();

  void initialize(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const Eigen::MatrixXd& Bd);

  void computeSteadyState(const Eigen::Vector3d &estimated_disturbance,
                          const Eigen::Matrix<double, kMeasurementSize, 1> &reference,
                          Eigen::Matrix<double, kStateSize, 1>* steadystate_state,
                          Eigen::Matrix<double, kInputSize, 1>* steadystate_input);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 private:
  ros::NodeHandle nh_, private_nh_, controller_nh_;
  bool initialized_params_;
  Eigen::Matrix<double, kStateSize, kInputSize> Bd_;
  Eigen::Matrix<double, kStateSize + kInputSize, kStateSize + kMeasurementSize> pseudo_inverse_left_hand_side_;

};

}

#endif /* INCLUDE_MAV_CONTROL_STEADYSTATECALCULATION_H_ */
