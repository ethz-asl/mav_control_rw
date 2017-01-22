/*
 Copyright (c) 2015, Mina Kamel, ASL, ETH Zurich, Switzerland
 Copyright (c) 2015, Markus Achtelik, ASL, ETH Zurich, Switzerland
 Copyright (c) 2015, Michael Burri, ASL, ETH Zurich, Switzerland

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

#include <mav_linear_mpc/linear_mpc.h>

namespace mav_control {

constexpr int LinearModelPredictiveController::kStateSize;
constexpr int LinearModelPredictiveController::kInputSize;
constexpr int LinearModelPredictiveController::kMeasurementSize;
constexpr int LinearModelPredictiveController::kDisturbanceSize;
constexpr double LinearModelPredictiveController::kGravity;
constexpr int LinearModelPredictiveController::kPredictionHorizonSteps;

LinearModelPredictiveController::LinearModelPredictiveController(const ros::NodeHandle& nh,
                                                                 const ros::NodeHandle& private_nh)
    : nh_(nh),
      private_nh_(private_nh),
      initialized_parameters_(false),
      position_error_integration_(0, 0, 0),
      command_roll_pitch_yaw_thrust_(0, 0, 0, 0),
      linearized_command_roll_pitch_thrust_(0, 0, 0),
      mpc_queue_(nh, private_nh, kPredictionHorizonSteps),
      disturbance_observer_(nh, private_nh),
      verbose_(false),
      solve_time_average_(0),
      steady_state_calculation_(nh, private_nh),
      received_first_odometry_(false)
{
  reset_integrator_service_server_ = nh_.advertiseService(
        "reset_integrator", &LinearModelPredictiveController::resetIntegratorServiceCallback, this);

  initializeParameters();

  mpc_queue_.initializeQueue(sampling_time_, prediction_sampling_time_);
}

LinearModelPredictiveController::~LinearModelPredictiveController()
{

}

bool LinearModelPredictiveController::resetIntegratorServiceCallback(std_srvs::Empty::Request &req,
                                                                     std_srvs::Empty::Response &res)
{
  position_error_integration_.setZero();
  return true;
}

void LinearModelPredictiveController::initializeParameters()
{
  std::vector<double> drag_coefficients;

  //Get parameters from RosParam server
  private_nh_.param<bool>("verbose", verbose_, false);

  if (!private_nh_.getParam("mass", mass_)) {
    ROS_ERROR("mass in MPC is not loaded from ros parameter server");
    abort();
  }

  if (!private_nh_.getParam("roll_time_constant", roll_time_constant_)) {
    ROS_ERROR("roll_time_constant in MPC is not loaded from ros parameter server");
    abort();
  }

  if (!private_nh_.getParam("pitch_time_constant", pitch_time_constant_)) {
    ROS_ERROR("pitch_time_constant in MPC is not loaded from ros parameter server");
    abort();
  }

  if (!private_nh_.getParam("roll_gain", roll_gain_)) {
    ROS_ERROR("roll_gain in MPC is not loaded from ros parameter server");
    abort();
  }

  if (!private_nh_.getParam("pitch_gain", pitch_gain_)) {
    ROS_ERROR("pitch_gain in MPC is not loaded from ros parameter server");
    abort();
  }

  if (!private_nh_.getParam("drag_coefficients", drag_coefficients)) {
    ROS_ERROR("drag_coefficients in MPC is not loaded from ros parameter server");
    abort();
  }

  drag_coefficients_ << drag_coefficients.at(0), drag_coefficients.at(1), drag_coefficients.at(2);

  if (!private_nh_.getParam("position_error_integration_limit",
                            position_error_integration_limit_)) {
    ROS_ERROR("position_error_integration_limit in MPC is not loaded from ros parameter server");
    abort();
  }

  if (!private_nh_.getParam("antiwindup_ball", antiwindup_ball_)) {
    ROS_ERROR("antiwindup_ball in MPC is not loaded from ros parameter server");
    abort();
  }

  if (!private_nh_.getParam("sampling_time", sampling_time_)) {
    ROS_ERROR("sampling_time in MPC is not loaded from ros parameter server");
    abort();
  }

  if (!private_nh_.getParam("prediction_sampling_time", prediction_sampling_time_)) {
    ROS_ERROR("prediction_sampling_time in MPC is not loaded from ros parameter server");
    abort();
  }

  //construct model matrices
  Eigen::MatrixXd A_continous_time(kStateSize, kStateSize);
  A_continous_time = Eigen::MatrixXd::Zero(kStateSize, kStateSize);
  Eigen::MatrixXd B_continous_time;
  B_continous_time = Eigen::MatrixXd::Zero(kStateSize, kInputSize);
  Eigen::MatrixXd Bd_continous_time;
  Bd_continous_time = Eigen::MatrixXd::Zero(kStateSize, kDisturbanceSize);

  A_continous_time(0, 3) = 1;
  A_continous_time(1, 4) = 1;
  A_continous_time(2, 5) = 1;
  A_continous_time(3, 3) = -drag_coefficients.at(0);
  A_continous_time(3, 7) = kGravity;
  A_continous_time(4, 4) = -drag_coefficients.at(1);
  A_continous_time(4, 6) = -kGravity;
  A_continous_time(5, 5) = -drag_coefficients.at(2);
  A_continous_time(6, 6) = -1.0 / roll_time_constant_;
  A_continous_time(7, 7) = -1.0 / pitch_time_constant_;

  B_continous_time(5, 2) = 1.0;
  B_continous_time(6, 0) = roll_gain_ / roll_time_constant_;
  B_continous_time(7, 1) = pitch_gain_ / pitch_time_constant_;

  Bd_continous_time(3, 0) = 1.0;
  Bd_continous_time(4, 1) = 1.0;
  Bd_continous_time(5, 2) = 1.0;

  model_A_ = (prediction_sampling_time_ * A_continous_time).exp();

  Eigen::MatrixXd integral_exp_A;
  integral_exp_A = Eigen::MatrixXd::Zero(kStateSize, kStateSize);
  const int count_integral_A = 100;

  for (int i = 0; i < count_integral_A; i++) {
    integral_exp_A += (A_continous_time * prediction_sampling_time_ * i / count_integral_A).exp()
        * prediction_sampling_time_ / count_integral_A;
  }

  model_B_ = integral_exp_A * B_continous_time;
  model_Bd_ = integral_exp_A * Bd_continous_time;

  steady_state_calculation_.initialize(model_A_, model_B_, model_Bd_);

  if (verbose_) {
    ROS_INFO_STREAM("A: \n" << model_A_);
    ROS_INFO_STREAM("B: \n" << model_B_);
    ROS_INFO_STREAM("B_d: \n" << model_Bd_);
  }

  //Solver initialization
  set_defaults();
  setup_indexing();

  //Solver settings
  settings.verbose = 0;

  Eigen::Map<Eigen::MatrixXd>(const_cast<double*>(params.A), kStateSize, kStateSize) = model_A_;
  Eigen::Map<Eigen::MatrixXd>(const_cast<double*>(params.B), kStateSize, kInputSize) = model_B_;
  Eigen::Map<Eigen::MatrixXd>(const_cast<double*>(params.Bd), kStateSize, kDisturbanceSize) =
      model_Bd_;

  initialized_parameters_ = true;

  ROS_INFO("Linear MPC: initialized correctly");
}

void LinearModelPredictiveController::applyParameters()
{
  Eigen::Matrix<double, kStateSize, kStateSize> Q;
  Eigen::Matrix<double, kStateSize, kStateSize> Q_final;
  Eigen::Matrix<double, kInputSize, kInputSize> R;
  Eigen::Matrix<double, kInputSize, kInputSize> R_delta;

  Q.setZero();
  Q_final.setZero();
  R.setZero();
  R_delta.setZero();

  Q.block(0, 0, 3, 3) = q_position_.asDiagonal();
  Q.block(3, 3, 3, 3) = q_velocity_.asDiagonal();
  Q.block(6, 6, 2, 2) = q_attitude_.asDiagonal();

  R = r_command_.asDiagonal();

  R_delta = r_delta_command_.asDiagonal();

  //Compute terminal cost
  //Q_final(k+1) = A'*Q_final(k)*A - (A'*Q_final(k)*B)*inv(B'*Q_final(k)*B+R)*(B'*Q_final(k)*A)+ Q;
  Q_final = Q;
  for (int i = 0; i < 1000; i++) {
    Eigen::MatrixXd temp = (model_B_.transpose() * Q_final * model_B_ + R);
    Q_final = model_A_.transpose() * Q_final * model_A_
        - (model_A_.transpose() * Q_final * model_B_) * temp.inverse()
            * (model_B_.transpose() * Q_final * model_A_) + Q;
  }

  Eigen::MatrixXd temporary_matrix = model_B_.transpose() * Q_final * model_B_ + R;
  LQR_K_ = temporary_matrix.inverse() * (model_B_.transpose() * Q_final * model_A_);

  Eigen::Map<Eigen::MatrixXd>(const_cast<double*>(params.Q), kStateSize, kStateSize) = Q;
  Eigen::Map<Eigen::MatrixXd>(const_cast<double*>(params.Q_final), kStateSize, kStateSize) =
      Q_final;
  Eigen::Map<Eigen::MatrixXd>(const_cast<double*>(params.R), kInputSize, kInputSize) = R;
  Eigen::Map<Eigen::MatrixXd>(const_cast<double*>(params.R_omega), kInputSize, kInputSize) = R_delta
      * (1.0 / sampling_time_ * sampling_time_);

  params.u_max[0] = roll_limit_;
  params.u_max[1] = pitch_limit_;
  params.u_max[2] = thrust_max_;

  params.u_min[0] = -roll_limit_;
  params.u_min[1] = -pitch_limit_;
  params.u_min[2] = thrust_min_;

  ROS_INFO("Linear MPC: Tuning parameters updated...");
  if (verbose_) {
    ROS_INFO_STREAM("diag(Q) = \n" << Q.diagonal().transpose());
    ROS_INFO_STREAM("diag(R) = \n" << R.diagonal().transpose());
    ROS_INFO_STREAM("diag(R_delta) = \n " << R_delta.diagonal().transpose());
    ROS_INFO_STREAM("Q_final = \n" << Q_final);
  }
}

void LinearModelPredictiveController::setOdometry(const mav_msgs::EigenOdometry& odometry)
{
  static mav_msgs::EigenOdometry previous_odometry = odometry;

  if (!received_first_odometry_) {
    Eigen::Vector3d euler_angles;
    odometry.getEulerAngles(&euler_angles);

    Eigen::VectorXd x0;

    disturbance_observer_.reset(odometry.position_W, odometry.getVelocityWorld(), euler_angles,
                                odometry.angular_velocity_B, Eigen::Vector3d::Zero(),
                                Eigen::Vector3d::Zero());

    received_first_odometry_ = true;
  }

  if (odometry.position_W.allFinite() == false) {
    odometry_.position_W = previous_odometry.position_W;
    ROS_WARN("Odometry.position has a non finite element");
  } else {
    odometry_.position_W = odometry.position_W;
    previous_odometry.position_W = odometry.position_W;
  }

  if (odometry.velocity_B.allFinite() == false) {
    odometry_.velocity_B = previous_odometry.velocity_B;
    ROS_WARN("Odometry.velocity has a non finite element");
  } else {
    odometry_.velocity_B = odometry.velocity_B;
    previous_odometry.velocity_B = odometry.velocity_B;
  }

  if (odometry.angular_velocity_B.allFinite() == false) {
    odometry_.angular_velocity_B = previous_odometry.angular_velocity_B;
    ROS_WARN("Odometry.angular_velocity has a non finite element");
  } else {
    odometry_.angular_velocity_B = odometry.angular_velocity_B;
    previous_odometry.angular_velocity_B = odometry.angular_velocity_B;
  }

  odometry_.orientation_W_B = odometry.orientation_W_B;
  previous_odometry.orientation_W_B = odometry.orientation_W_B;
}

void LinearModelPredictiveController::setCommandTrajectoryPoint(
    const mav_msgs::EigenTrajectoryPoint& command_trajectory)
{
  mpc_queue_.insertReference(command_trajectory);
}

void LinearModelPredictiveController::setCommandTrajectory(
    const mav_msgs::EigenTrajectoryPointDeque& command_trajectory_array)
{
  int array_size = command_trajectory_array.size();
  if (array_size < 1) {
    return;
  }

  mpc_queue_.insertReferenceTrajectory(command_trajectory_array);
}

void LinearModelPredictiveController::calculateRollPitchYawrateThrustCommand(
    Eigen::Vector4d *ref_attitude_thrust)
{
  assert(ref_attitude_thrust != nullptr);
  assert(initialized_parameters_ == true);
  ros::WallTime starting_time = ros::WallTime::now();

  //Declare variables
  Eigen::Matrix<double, kMeasurementSize, 1> reference;
  Eigen::VectorXd KF_estimated_state;
  Eigen::Vector2d roll_pitch_inertial_frame;
  Eigen::Matrix<double, kDisturbanceSize, 1> estimated_disturbances;
  Eigen::Matrix<double, kStateSize, 1> x_0;

  Eigen::Vector3d current_rpy;
  odometry_.getEulerAngles(&current_rpy);

  double roll;
  double pitch;
  double yaw;

  // update mpc queue
  mpc_queue_.updateQueue();
  // Copy out the whole queues
  mpc_queue_.getQueue(position_ref_, velocity_ref_, acceleration_ref_, yaw_ref_, yaw_rate_ref_);

  // update the disturbance observer
  disturbance_observer_.feedAttitudeCommand(command_roll_pitch_yaw_thrust_);
  disturbance_observer_.feedPositionMeasurement(odometry_.position_W);
  disturbance_observer_.feedVelocityMeasurement(odometry_.getVelocityWorld());
  disturbance_observer_.feedRotationMatrix(odometry_.orientation_W_B.toRotationMatrix());

  bool observer_update_successful = disturbance_observer_.updateEstimator();

  if (!observer_update_successful) {
    // reset the disturbance observer
    disturbance_observer_.reset(odometry_.position_W, odometry_.getVelocityWorld(), current_rpy,
                                Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                                Eigen::Vector3d::Zero());
  }

  disturbance_observer_.getEstimatedState(&KF_estimated_state);

  if (enable_offset_free_ == true) {
    estimated_disturbances = KF_estimated_state.segment(12, kDisturbanceSize);
  } else {
    estimated_disturbances.setZero();
  }

  if (enable_integrator_) {
    Eigen::Vector3d position_error = position_ref_.front() - odometry_.position_W;
    if (position_error.norm() < antiwindup_ball_) {
      position_error_integration_ += position_error * sampling_time_;
    } else {
      position_error_integration_.setZero();
    }

    position_error_integration_ = position_error_integration_.cwiseMax(
        Eigen::Vector3d(-position_error_integration_limit_, -position_error_integration_limit_,
                        -position_error_integration_limit_));

    position_error_integration_ = position_error_integration_.cwiseMin(
        Eigen::Vector3d(position_error_integration_limit_, position_error_integration_limit_,
                        position_error_integration_limit_));

    estimated_disturbances -= Eigen::Vector3d(Ki_xy_, Ki_xy_, Ki_altitude_).asDiagonal()
        * position_error_integration_;
  }


  Eigen::Matrix<double, kStateSize, 1> target_state;
  Eigen::Matrix<double, kInputSize, 1> target_input;
  Eigen::VectorXd ref(kMeasurementSize);

  CVXGEN_queue_.clear();
  for (int i = 0; i < kPredictionHorizonSteps - 1; i++) {
    ref << position_ref_.at(i), velocity_ref_.at(i);
    steady_state_calculation_.computeSteadyState(estimated_disturbances, ref, &target_state,
                                                 &target_input);
    CVXGEN_queue_.push_back(target_state);
    if (i == 0) {
      Eigen::Map<Eigen::Matrix<double, kInputSize, 1>>(const_cast<double*>(params.u_ss)) =
          target_input;
    }
  }
  ref << position_ref_.at(kPredictionHorizonSteps - 1), velocity_ref_.at(
      kPredictionHorizonSteps - 1);

  steady_state_calculation_.computeSteadyState(estimated_disturbances, ref, &target_state,
                                               &target_input);
  CVXGEN_queue_.push_back(target_state);

  for (int i = 0; i < kPredictionHorizonSteps; i++) {
    Eigen::Map<Eigen::VectorXd>(const_cast<double*>(params.x_ss[i]), kStateSize, 1) =
        CVXGEN_queue_[i];
  }

  roll = current_rpy(0);
  pitch = current_rpy(1);
  yaw = current_rpy(2);

  roll_pitch_inertial_frame << -sin(yaw) * pitch + cos(yaw) * roll, cos(yaw) * pitch
      + sin(yaw) * roll;
  x_0 << odometry_.position_W, odometry_.getVelocityWorld(), roll_pitch_inertial_frame;

  //Solve using CVXGEN
  Eigen::Map<Eigen::Matrix<double, kDisturbanceSize, 1>>(const_cast<double*>(params.d)) =
      estimated_disturbances;
  Eigen::Map<Eigen::Matrix<double, kInputSize, 1>>(const_cast<double*>(params.u_prev)) =
      linearized_command_roll_pitch_thrust_;
  Eigen::Map<Eigen::Matrix<double, kStateSize, 1>>(const_cast<double*>(params.x_0)) = x_0;

  tic();
  int solver_status = solve();
  solve_time_average_ += tocq();

  linearized_command_roll_pitch_thrust_ << vars.u_0[0], vars.u_0[1], vars.u_0[2];

  if (solver_status < 0) {
    ROS_WARN("Linear MPC: Solver faild, use LQR backup");
    linearized_command_roll_pitch_thrust_ = LQR_K_ * (target_state - x_0);
    linearized_command_roll_pitch_thrust_ = linearized_command_roll_pitch_thrust_.cwiseMax(
        Eigen::Vector3d(-roll_limit_, -pitch_limit_, thrust_min_));
    linearized_command_roll_pitch_thrust_ = linearized_command_roll_pitch_thrust_.cwiseMin(
        Eigen::Vector3d(roll_limit_, pitch_limit_, thrust_max_));
  }

  command_roll_pitch_yaw_thrust_(3) = (linearized_command_roll_pitch_thrust_(2) + kGravity)
      / (cos(roll) * cos(pitch));
  double ux = linearized_command_roll_pitch_thrust_(1)
      * (kGravity / command_roll_pitch_yaw_thrust_(3));
  double uy = linearized_command_roll_pitch_thrust_(0)
      * (kGravity / command_roll_pitch_yaw_thrust_(3));

  command_roll_pitch_yaw_thrust_(0) = ux * sin(yaw) + uy * cos(yaw);
  command_roll_pitch_yaw_thrust_(1) = ux * cos(yaw) - uy * sin(yaw);
  command_roll_pitch_yaw_thrust_(2) = yaw_ref_.front();

  // yaw controller
  double yaw_error = command_roll_pitch_yaw_thrust_(2) - yaw;

  if (std::abs(yaw_error) > M_PI) {
    if (yaw_error > 0.0) {
      yaw_error = yaw_error - 2.0 * M_PI;
    } else {
      yaw_error = yaw_error + 2.0 * M_PI;
    }
  }

  double yaw_rate_cmd = K_yaw_ * yaw_error + yaw_rate_ref_.front(); // feed-forward yaw_rate cmd

  if (yaw_rate_cmd > yaw_rate_limit_) {
    yaw_rate_cmd = yaw_rate_limit_;
  }

  if (yaw_rate_cmd < -yaw_rate_limit_) {
    yaw_rate_cmd = -yaw_rate_limit_;
  }

  *ref_attitude_thrust = Eigen::Vector4d(command_roll_pitch_yaw_thrust_(0),
                                         command_roll_pitch_yaw_thrust_(1), yaw_rate_cmd,
                                         command_roll_pitch_yaw_thrust_(3) * mass_);  //[N]

  double diff_time = (ros::WallTime::now() - starting_time).toSec();

  if (verbose_) {
    static int counter = 0;
    if (counter > 100) {
      ROS_INFO_STREAM("average solve time: " << 1000.0 * solve_time_average_ / counter << " ms");
      solve_time_average_ = 0.0;

      ROS_INFO_STREAM("Controller loop time : " << diff_time * 1000.0 << " ms");

      ROS_INFO_STREAM(
          "roll ref: " << command_roll_pitch_yaw_thrust_(0)
          << "\t" << "pitch ref : \t" << command_roll_pitch_yaw_thrust_(1)
          << "\t" << "yaw ref : \t" << command_roll_pitch_yaw_thrust_(2)
          << "\t" << "thrust ref : \t" << command_roll_pitch_yaw_thrust_(3)
          << "\t" << "yawrate ref : \t" << yaw_rate_cmd);
      counter = 0;
    }
    counter++;
  }
}

bool LinearModelPredictiveController::getCurrentReference(
    mav_msgs::EigenTrajectoryPoint* reference) const
{
  assert(reference != nullptr);

  (*reference).position_W = position_ref_.front();
  (*reference).velocity_W = velocity_ref_.front();
  (*reference).setFromYaw(yaw_ref_.front());

  return true;
}

bool LinearModelPredictiveController::getCurrentReference(
    mav_msgs::EigenTrajectoryPointDeque* reference) const
{
  assert(reference != nullptr);

  (*reference).clear();

  for (int i = 0; i < position_ref_.size(); i++) {
    mav_msgs::EigenTrajectoryPoint pnt;
    pnt.position_W = position_ref_.at(i);
    pnt.setFromYaw(yaw_ref_.at(i));
    (*reference).push_back(pnt);
  }
  return true;
}

bool LinearModelPredictiveController::getPredictedState(
    mav_msgs::EigenTrajectoryPointDeque* predicted_state) const
{
  assert(predicted_state != nullptr);

  for (size_t i = 1; i < kPredictionHorizonSteps; i++) {
    mav_msgs::EigenTrajectoryPoint pnt;
    pnt.position_W = Eigen::Vector3d(vars.x[i][0], vars.x[i][1], vars.x[i][2]);
    (*predicted_state).push_back(pnt);
  }

  return true;
}

}
