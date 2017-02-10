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

#include <mav_linear_mpc/steady_state_calculation.h>

namespace mav_control {
SteadyStateCalculation::SteadyStateCalculation(const ros::NodeHandle& nh,
                                               const ros::NodeHandle& private_nh)
    : nh_(nh),
      private_nh_(private_nh),
      controller_nh_(private_nh, "controller"),
      initialized_params_(false)
{

}

SteadyStateCalculation::~SteadyStateCalculation()
{
}

void SteadyStateCalculation::initialize(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B,
                                        const Eigen::MatrixXd& Bd)
{
  Eigen::MatrixXd left_hand_side;
  left_hand_side.resize(kStateSize + kMeasurementSize, kStateSize + kInputSize);

  Bd_ = Bd;
  Eigen::MatrixXd C(6, 8);
  C.setIdentity();

  left_hand_side << A - Eigen::MatrixXd::Identity(kStateSize, kStateSize), B, C, Eigen::MatrixXd::Zero(
      kMeasurementSize, kInputSize);

  pseudo_inverse_left_hand_side_ = (left_hand_side.transpose() * left_hand_side).inverse()
      * left_hand_side.transpose();

  initialized_params_ = true;
  ROS_INFO("Linear MPC: Steady State calculation is initialized correctly");
}

void SteadyStateCalculation::computeSteadyState(
    const Eigen::Vector3d &estimated_disturbance,
    const Eigen::Matrix<double, kMeasurementSize, 1> &reference,
    Eigen::Matrix<double, kStateSize, 1>* steadystate_state,
    Eigen::Matrix<double, kInputSize, 1>* steadystate_input)
{
  assert(steadystate_state);
  assert(steadystate_input);
  assert(initialized_params_);

  Eigen::Matrix<double, kStateSize + kInputSize, 1> target_state_and_input;
  Eigen::Matrix<double, kStateSize + kMeasurementSize, 1> right_hand_side;

  right_hand_side << -Bd_ * estimated_disturbance, reference;

  target_state_and_input = pseudo_inverse_left_hand_side_ * right_hand_side;

  *steadystate_state = target_state_and_input.segment(0, kStateSize);
  *steadystate_input = target_state_and_input.segment(kStateSize, kInputSize);
}

}
