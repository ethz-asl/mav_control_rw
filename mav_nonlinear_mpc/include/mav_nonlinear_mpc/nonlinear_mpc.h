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

#ifndef INCLUDE_MAV_NONLINEAR_MPC_NONLINEAR_MPC_H_
#define INCLUDE_MAV_NONLINEAR_MPC_NONLINEAR_MPC_H_

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <mav_msgs/conversions.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <stdio.h>
#include <mav_control_interface/mpc_queue.h>
#include "acado_common.h"
#include "acado_auxiliary_functions.h"
#include <mav_disturbance_observer/KF_disturbance_observer.h>
#include <std_srvs/Empty.h>
#include <lapacke.h>

ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;

namespace mav_control {

lapack_logical select_lhp(const double *real, const double *imag)
{
  return *real < 0.0;
}

class NonlinearModelPredictiveControl
{
 public:
  NonlinearModelPredictiveControl(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh);
  ~NonlinearModelPredictiveControl();

  // Dynamic parameters
  void setPositionPenality(const Eigen::Vector3d& q_position)
  {
    q_position_ = q_position;
  }
  void setVelocityPenality(const Eigen::Vector3d& q_velocity)
  {
    q_velocity_ = q_velocity;
  }
  void setAttitudePenality(const Eigen::Vector2d& q_attitude)
  {
    q_attitude_ = q_attitude;
  }
  void setCommandPenality(const Eigen::Vector3d& r_command)
  {
    r_command_ = r_command;
  }
  void setYawGain(double K_yaw)
  {
    K_yaw_ = K_yaw;
  }

  void setAltitudeIntratorGain(double Ki_altitude)
  {
    Ki_altitude_ = Ki_altitude;
  }

  void setXYIntratorGain(double Ki_xy)
  {
    Ki_xy_ = Ki_xy;
  }

  void setEnableOffsetFree(bool enable_offset_free)
  {
    enable_offset_free_ = enable_offset_free;
  }

  void setEnableIntegrator(bool enable_integrator)
  {
    enable_integrator_ = enable_integrator;
  }

  void setControlLimits(const Eigen::VectorXd& control_limits)
  {
    //roll_max, pitch_max, yaw_rate_max, thrust_min and thrust_max
    roll_limit_ = control_limits(0);
    pitch_limit_ = control_limits(1);
    yaw_rate_limit_ = control_limits(2);
    thrust_min_ = control_limits(3);
    thrust_max_ = control_limits(4);
  }

  void applyParameters();

  double getMass() const
  {
    return mass_;
  }

  double getThrustMin() const
  {
    return thrust_min_;
  }

  double getThrustMax() const
  {
    return thrust_max_;
  }

  // get reference and predicted state
  bool getCurrentReference(mav_msgs::EigenTrajectoryPoint* reference) const;
  bool getCurrentReference(mav_msgs::EigenTrajectoryPointDeque* reference) const;
  bool getPredictedState(mav_msgs::EigenTrajectoryPointDeque* predicted_state) const;

  // set odom and commands
  void setOdometry(const mav_msgs::EigenOdometry& odometry);
  void setCommandTrajectoryPoint(const mav_msgs::EigenTrajectoryPoint& command_trajectory);
  void setCommandTrajectory(const mav_msgs::EigenTrajectoryPointDeque& command_trajectory);

  // compute control input
  void calculateRollPitchYawrateThrustCommand(Eigen::Vector4d* ref_attitude_thrust);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 private:

  // constants
  static constexpr double kGravity = 9.8066;
  static constexpr int kDisturbanceSize = 3;

  // ros node handles
  ros::NodeHandle nh_, private_nh_;

  // reset integrator service
  ros::ServiceServer reset_integrator_service_server_;
  bool resetIntegratorServiceCallback(std_srvs::Empty::Request &req,
                                      std_srvs::Empty::Response &res);

  // sampling time parameters
  void initializeParameters();
  bool initialized_parameters_;

  // sampling time parameters
  double sampling_time_;
  double prediction_sampling_time_;

  // system model parameters
  double mass_;
  double roll_time_constant_;
  double roll_gain_;
  double pitch_time_constant_;
  double pitch_gain_;
  Eigen::Vector3d drag_coefficients_;

  // controller parameters
  // state penalty
  Eigen::Vector3d q_position_;
  Eigen::Vector3d q_velocity_;
  Eigen::Vector2d q_attitude_;

  // control penalty
  Eigen::Vector3d r_command_;

  // yaw P gain
  double K_yaw_;

  // error integrator
  bool enable_integrator_;
  double Ki_altitude_;
  double Ki_xy_;
  double antiwindup_ball_;
  Eigen::Vector3d position_error_integration_;
  double position_error_integration_limit_;

  // control input limits
  double roll_limit_;
  double pitch_limit_;
  double yaw_rate_limit_;
  double thrust_min_;
  double thrust_max_;

  // reference queue
  MPCQueue mpc_queue_;
  Vector3dDeque position_ref_, velocity_ref_, acceleration_ref_;
  std::deque<double> yaw_ref_, yaw_rate_ref_;

  // solver matrices
  Eigen::Matrix<double, ACADO_NY, ACADO_NY> W_;
  Eigen::Matrix<double, ACADO_NYN, ACADO_NYN> WN_;
  Eigen::Matrix<double, ACADO_N + 1, ACADO_NX> state_;
  Eigen::Matrix<double, ACADO_N, ACADO_NU> input_;
  Eigen::Matrix<double, ACADO_N, ACADO_NY> reference_;
  Eigen::Matrix<double, 1, ACADO_NYN> referenceN_;
  Eigen::Matrix<double, ACADO_N + 1, ACADO_NOD> acado_online_data_;

  // disturbance observer
  bool enable_offset_free_;
  KFDisturbanceObserver disturbance_observer_;

  // commands
  Eigen::Vector4d command_roll_pitch_yaw_thrust_;

  // debug info
  bool verbose_;
  double solve_time_average_;

  // most recent odometry information
  mav_msgs::EigenOdometry odometry_;
  bool received_first_odometry_;

  // initilize solver
  void initializeAcadoSolver(Eigen::VectorXd x0);

  // solve continuous time Riccati equation
  Eigen::MatrixXd solveCARE(Eigen::MatrixXd Q, Eigen::MatrixXd R);

};

}

#endif /* INCLUDE_MAV_NONLINEAR_MPC_NONLINEAR_MPC_H_ */
