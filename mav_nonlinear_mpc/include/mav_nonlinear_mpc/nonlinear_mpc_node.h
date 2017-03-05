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
#ifndef INCLUDE_MAV_NONLINEAR_MPC_NONLINEAR_MPC_NODE_H_
#define INCLUDE_MAV_NONLINEAR_MPC_NONLINEAR_MPC_NODE_H_

#include <boost/bind.hpp>
#include <Eigen/Eigen>
#include <stdio.h>

//ros
#include <ros/ros.h>
#include <ros/callback_queue.h>

//ros msgs
#include <mav_msgs/RollPitchYawrateThrust.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <mav_msgs/conversions.h>
#include <nav_msgs/Odometry.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <mav_msgs/Status.h>

//dynamic reconfiguration
#include <dynamic_reconfigure/server.h>
#include <mav_nonlinear_mpc/NonLinearMPCConfig.h>

#include <mav_nonlinear_mpc/nonlinear_mpc.h>
#include <mav_control_interface/position_controller_interface.h>

namespace mav_control {

class NonLinearModelPredictiveControllerNode : public mav_control_interface::PositionControllerInterface
{
 public:
  NonLinearModelPredictiveControllerNode(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh);
  ~NonLinearModelPredictiveControllerNode();

  void InitializeParams();

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 private:
  NonlinearModelPredictiveControl nonlinear_mpc_;

  dynamic_reconfigure::Server<mav_nonlinear_mpc::NonLinearMPCConfig> controller_dyn_config_server_;

  void ControllerDynConfigCallback(mav_nonlinear_mpc::NonLinearMPCConfig &config, uint32_t level);

  virtual std::string getName() const
  {
    return std::string("nonlinear_model_predictive_controller");
  }

  virtual bool getUseAttitudeQuaternionCommand() const
  {
    return false;
  }

  virtual double getMass() const {
    return nonlinear_mpc_.getMass();
  }

  virtual double getThrustMin() const {
    return nonlinear_mpc_.getThrustMin();
  }

  virtual double getThrustMax() const {
    return nonlinear_mpc_.getThrustMax();
  }

  virtual bool setReference(const mav_msgs::EigenTrajectoryPoint& reference);

  virtual bool setReferenceArray(const mav_msgs::EigenTrajectoryPointDeque& reference_array);

  virtual bool setOdometry(const mav_msgs::EigenOdometry& odometry);

  virtual bool calculateRollPitchYawrateThrustCommand(
      mav_msgs::EigenRollPitchYawrateThrust* attitude_thrust_command);

  virtual bool calculateAttitudeThrustCommand(mav_msgs::EigenAttitudeThrust* attitude_thrust_command);

  virtual bool getCurrentReference(mav_msgs::EigenTrajectoryPoint* reference) const;

  virtual bool getCurrentReference(mav_msgs::EigenTrajectoryPointDeque* reference) const;

  virtual bool getPredictedState(mav_msgs::EigenTrajectoryPointDeque* predicted_state) const;

  void uavStatusCallback(const mav_msgs::StatusConstPtr& msg);

};

}


#endif /* INCLUDE_MAV_NONLINEAR_MPC_NONLINEAR_MPC_NODE_H_ */
