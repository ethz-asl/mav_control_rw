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

#include <mav_nonlinear_mpc/nonlinear_mpc.h>

namespace mav_control {

constexpr double NonlinearModelPredictiveControl::kGravity;
constexpr int NonlinearModelPredictiveControl::kDisturbanceSize;

NonlinearModelPredictiveControl::NonlinearModelPredictiveControl(const ros::NodeHandle& nh,
                                                                 const ros::NodeHandle& private_nh)
    : nh_(nh),
      private_nh_(private_nh),
      initialized_parameters_(false),
      position_error_integration_(0, 0, 0),
      mpc_queue_(nh, private_nh, ACADO_N),
      command_roll_pitch_yaw_thrust_(0, 0, 0, 0),
      disturbance_observer_(nh, private_nh),
      verbose_(false),
      solve_time_average_(0),
      received_first_odometry_(false)
{

  acado_initializeSolver();

  W_.setZero();
  WN_.setZero();

  input_.setZero();
  state_.setZero();
  reference_.setZero();
  referenceN_.setZero();

  reset_integrator_service_server_ = nh_.advertiseService(
      "reset_integrator", &NonlinearModelPredictiveControl::resetIntegratorServiceCallback, this);

  initializeParameters();

  mpc_queue_.initializeQueue(sampling_time_, prediction_sampling_time_);

}

NonlinearModelPredictiveControl::~NonlinearModelPredictiveControl()
{

}

bool NonlinearModelPredictiveControl::resetIntegratorServiceCallback(std_srvs::Empty::Request &req,
                                                                     std_srvs::Empty::Response &res)
{
  position_error_integration_.setZero();
  return true;
}

void NonlinearModelPredictiveControl::initializeParameters()
{
  std::vector<double> drag_coefficients;

  //Get parameters from RosParam server
  private_nh_.param<bool>("verbose", verbose_, false);

  if (!private_nh_.getParam("mass", mass_)) {
    ROS_ERROR("mass in nonlinear MPC controller is not loaded from ros parameter "
              "server");
    abort();
  }

  if (!private_nh_.getParam("roll_time_constant", roll_time_constant_)) {
    ROS_ERROR(
        "roll_time_constant in nonlinear MPC controller is not loaded from ros parameter server");
    abort();
  }

  if (!private_nh_.getParam("roll_gain", roll_gain_)) {
    ROS_ERROR("roll_gain in nonlinear MPC controller is not loaded from ros parameter server");
    abort();
  }

  if (!private_nh_.getParam("pitch_time_constant", pitch_time_constant_)) {
    ROS_ERROR(
        "pitch_time_constant in nonlinear MPC controller is not loaded from ros parameter server");
    abort();
  }

  if (!private_nh_.getParam("pitch_gain", pitch_gain_)) {
    ROS_ERROR("pitch_gain in nonlinear MPC controller is not loaded from ros parameter server");
    abort();
  }

  if (!private_nh_.getParam("linear_drag_coefficients", drag_coefficients)) {
    ROS_ERROR(
        "linear_drag_coefficients in nonlinear MPC controller is not loaded from ros parameter server");
    abort();
  }

  drag_coefficients_ << drag_coefficients.at(0), drag_coefficients.at(1), drag_coefficients.at(2);

  if (!private_nh_.getParam("antiwindup_ball", antiwindup_ball_)) {
    ROS_ERROR(
        "antiwindup_ball in nonlinear MPC controller is not loaded from ros parameter server");
    abort();
  }

  if (!private_nh_.getParam("position_error_integration_limit",
                            position_error_integration_limit_)) {
    ROS_ERROR(
        "position_error_integration_limit in nonlinear MPC is not loaded from ros parameter server");
    abort();
  }

  if (!private_nh_.getParam("sampling_time", sampling_time_)) {
    ROS_ERROR("sampling_time in nonlinear MPC is not loaded from ros parameter server");
    abort();
  }

  if (!private_nh_.getParam("prediction_sampling_time", prediction_sampling_time_)) {
    ROS_ERROR("prediction_sampling_time in nonlinear MPC is not loaded from ros parameter server");
    abort();
  }

  for (int i = 0; i < ACADO_N + 1; i++) {
    acado_online_data_.block(i, 0, 1, ACADO_NOD) << roll_time_constant_, roll_gain_, pitch_time_constant_, pitch_gain_, drag_coefficients_(
        0), drag_coefficients_(1), 0, 0, 0;
  }

  Eigen::Map<Eigen::Matrix<double, ACADO_NOD, ACADO_N + 1>>(const_cast<double*>(acadoVariables.od)) =
      acado_online_data_.transpose();

  if (verbose_) {
    std::cout << "acado online data: " << std::endl << acado_online_data_ << std::endl;
  }

  initialized_parameters_ = true;
  ROS_INFO("Nonlinear MPC: initialized correctly");
}

void NonlinearModelPredictiveControl::applyParameters()
{
  W_.block(0, 0, 3, 3) = q_position_.asDiagonal();
  W_.block(3, 3, 3, 3) = q_velocity_.asDiagonal();
  W_.block(6, 6, 2, 2) = q_attitude_.asDiagonal();
  W_.block(8, 8, 3, 3) = r_command_.asDiagonal();

  WN_ = solveCARE((Eigen::VectorXd(6) << q_position_, q_velocity_).finished().asDiagonal(),
                  r_command_.asDiagonal());

  Eigen::Map<Eigen::Matrix<double, ACADO_NY, ACADO_NY>>(const_cast<double*>(acadoVariables.W)) = W_
      .transpose();
  Eigen::Map<Eigen::Matrix<double, ACADO_NYN, ACADO_NYN>>(const_cast<double*>(acadoVariables.WN)) =
      WN_.transpose();

  for (size_t i = 0; i < ACADO_N; ++i) {
    acadoVariables.lbValues[3 * i] = -roll_limit_;       // min roll
    acadoVariables.lbValues[3 * i + 1] = -pitch_limit_;  // min pitch
    acadoVariables.lbValues[3 * i + 2] = thrust_min_;    // min thrust
    acadoVariables.ubValues[3 * i] = roll_limit_;        // max roll
    acadoVariables.ubValues[3 * i + 1] = pitch_limit_;   // max pitch
    acadoVariables.ubValues[3 * i + 2] = thrust_max_;    // max thrust
  }

  if (verbose_) {
    std::cout << "q_position_: " << q_position_.transpose() << std::endl;
    std::cout << "q_velocity_: " << q_velocity_.transpose() << std::endl;
    std::cout << "r_command_: " << r_command_.transpose() << std::endl;
    std::cout << "W_N = \n" << WN_ << std::endl;
  }
}

void NonlinearModelPredictiveControl::setOdometry(const mav_msgs::EigenOdometry& odometry)
{
  static mav_msgs::EigenOdometry previous_odometry = odometry;

  if (!received_first_odometry_) {
    Eigen::Vector3d euler_angles;
    odometry.getEulerAngles(euler_angles);

    Eigen::VectorXd x0(ACADO_NX);

    x0 << odometry.getVelocityWorld(), euler_angles, odometry.position_W;

    initializeAcadoSolver(x0);

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

void NonlinearModelPredictiveControl::setCommandTrajectoryPoint(
    const mav_msgs::EigenTrajectoryPoint& command_trajectory)
{
  mpc_queue_.insertReference(command_trajectory);
}

void NonlinearModelPredictiveControl::setCommandTrajectory(
    const mav_msgs::EigenTrajectoryPointDeque& command_trajectory)
{
  int array_size = command_trajectory.size();
  if (array_size < 1)
    return;

  mpc_queue_.insertReferenceTrajectory(command_trajectory);
}

void NonlinearModelPredictiveControl::initializeAcadoSolver(Eigen::VectorXd x0)
{
  for (int i = 0; i < ACADO_N + 1; i++) {
    state_.block(i, 0, 1, ACADO_NX) << x0.transpose();
  }

  Eigen::Map<Eigen::Matrix<double, ACADO_NX, ACADO_N + 1>>(const_cast<double*>(acadoVariables.x)) =
      state_.transpose();
  Eigen::Map<Eigen::Matrix<double, ACADO_NU, ACADO_N>>(const_cast<double*>(acadoVariables.u)) =
      input_.transpose();
  Eigen::Map<Eigen::Matrix<double, ACADO_NY, ACADO_N>>(const_cast<double*>(acadoVariables.y)) =
      reference_.transpose();
  Eigen::Map<Eigen::Matrix<double, ACADO_NYN, 1>>(const_cast<double*>(acadoVariables.yN)) =
      referenceN_.transpose();
}

void NonlinearModelPredictiveControl::calculateRollPitchYawrateThrustCommand(
    Eigen::Vector4d* ref_attitude_thrust)
{
  assert(ref_attitude_thrust != nullptr);
  assert(initialized_parameters_ == true);
  ros::WallTime starting_time = ros::WallTime::now();

  Eigen::VectorXd KF_estimated_state;
  Eigen::Vector3d estimated_disturbances;
  Eigen::Matrix<double, ACADO_NX, 1> x_0;

  Eigen::Vector3d current_rpy;
  odometry_.getEulerAngles(current_rpy);

  mpc_queue_.updateQueue();
  mpc_queue_.getQueue(position_ref_, velocity_ref_, acceleration_ref_, yaw_ref_, yaw_rate_ref_);

  disturbance_observer_.feedAttitudeCommand(command_roll_pitch_yaw_thrust_);
  disturbance_observer_.feedPositionMeasurement(odometry_.position_W);
  disturbance_observer_.feedVelocityMeasurement(odometry_.getVelocityWorld());
  disturbance_observer_.feedRotationMatrix(odometry_.orientation_W_B.toRotationMatrix());

  bool observer_update_successful = disturbance_observer_.updateEstimator();
  if (!observer_update_successful) {
    disturbance_observer_.reset(odometry_.position_W, odometry_.getVelocityWorld(), current_rpy,
                                Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                                Eigen::Vector3d::Zero());
  }

  disturbance_observer_.getEstimatedState(&KF_estimated_state);

  if (enable_offset_free_ == true) {
    estimated_disturbances = KF_estimated_state.segment(12, kDisturbanceSize);
  } else {
    estimated_disturbances.setZero(kDisturbanceSize);
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

  double current_yaw = odometry_.getYaw();

  Eigen::Vector3d estimated_disturbances_B =
      odometry_.orientation_W_B.toRotationMatrix().transpose() * estimated_disturbances;

  for (size_t i = 0; i < ACADO_N; i++) {
    Eigen::Vector3d acceleration_ref_B = odometry_.orientation_W_B.toRotationMatrix().transpose()
        * acceleration_ref_[i];

    Eigen::Vector2d feed_forward(
        ((estimated_disturbances_B(1) - acceleration_ref_B(1)) / kGravity),
        ((-estimated_disturbances_B(0) + acceleration_ref_B(0)) / kGravity));
    reference_.block(i, 0, 1, ACADO_NY) << position_ref_[i].transpose(), velocity_ref_[i].transpose(), feed_forward
        .transpose(), feed_forward.transpose(), acceleration_ref_[i].z();
    acado_online_data_.block(i, ACADO_NOD - 3, 1, 3) << estimated_disturbances.transpose();
  }
  referenceN_ << position_ref_[ACADO_N].transpose(), velocity_ref_[ACADO_N].transpose();

  x_0 << odometry_.getVelocityWorld(), current_rpy, odometry_.position_W;

  Eigen::Map<Eigen::Matrix<double, ACADO_NX, 1>>(const_cast<double*>(acadoVariables.x0)) = x_0;
  Eigen::Map<Eigen::Matrix<double, ACADO_NY, ACADO_N>>(const_cast<double*>(acadoVariables.y)) =
      reference_.transpose();
  Eigen::Map<Eigen::Matrix<double, ACADO_NYN, 1>>(const_cast<double*>(acadoVariables.yN)) =
      referenceN_.transpose();
  Eigen::Map<Eigen::Matrix<double, ACADO_NOD, ACADO_N + 1>>(const_cast<double*>(acadoVariables.od)) =
      acado_online_data_.transpose();

  ros::WallTime time_before_solving = ros::WallTime::now();

  acado_preparationStep();

  int acado_status = acado_feedbackStep();

  solve_time_average_ += (ros::WallTime::now() - time_before_solving).toSec() * 1000.0;

  double roll_ref = acadoVariables.u[0];
  double pitch_ref = acadoVariables.u[1];
  double thrust_ref = acadoVariables.u[2];

  if (std::isnan(roll_ref) || std::isnan(pitch_ref) || std::isnan(thrust_ref)
      || acado_status != 0) {
    ROS_WARN_STREAM("Nonlinear MPC: Solver failed with status: " << acado_status);
    ROS_WARN("reinitializing...");
    initializeAcadoSolver (x_0);
    *ref_attitude_thrust << 0, 0, 0, kGravity * mass_;
    return;
  }

  command_roll_pitch_yaw_thrust_ << roll_ref, pitch_ref, yaw_ref_.front(), thrust_ref;

  state_ = Eigen::Map<Eigen::Matrix<double, ACADO_N + 1, ACADO_NX, Eigen::RowMajor>>(
      acadoVariables.x);

  // yaw controller
  double yaw_error = yaw_ref_.front() - current_yaw;

  if (std::abs(yaw_error) > M_PI) {
    if (yaw_error > 0.0) {
      yaw_error = yaw_error - 2.0 * M_PI;
    } else {
      yaw_error = yaw_error + 2.0 * M_PI;
    }
  }

  double yaw_rate_cmd = K_yaw_ * yaw_error + yaw_rate_ref_.front();  // feed-forward yaw_rate cmd

  if (yaw_rate_cmd > yaw_rate_limit_) {
    yaw_rate_cmd = yaw_rate_limit_;
  }

  if (yaw_rate_cmd < -yaw_rate_limit_) {
    yaw_rate_cmd = -yaw_rate_limit_;
  }

  *ref_attitude_thrust = Eigen::Vector4d(roll_ref, pitch_ref, yaw_rate_cmd, mass_ * thrust_ref);

  double diff_time = (ros::WallTime::now() - starting_time).toSec();

  if (verbose_) {
    static int counter = 0;
    if (counter > 100) {
      ROS_INFO_STREAM("average solve time: " << solve_time_average_ / counter);
      solve_time_average_ = 0.0;

      ROS_INFO_STREAM("Controller loop time : " << diff_time << " sec");

      ROS_INFO_STREAM(
          "roll ref: " << command_roll_pitch_yaw_thrust_(0) << "\t" << "pitch ref : \t" << command_roll_pitch_yaw_thrust_(1) << "\t" << "yaw ref : \t" << command_roll_pitch_yaw_thrust_(2) << "\t" << "thrust ref : \t" << command_roll_pitch_yaw_thrust_(3) << "\t" << "yawrate ref : \t" << yaw_rate_cmd);
      counter = 0;
    }
    counter++;
  }

}

Eigen::MatrixXd NonlinearModelPredictiveControl::solveCARE(Eigen::MatrixXd Q, Eigen::MatrixXd R)
{
  // Define system matrices
  Eigen::MatrixXd A;
  A.resize(6, 6);
  A.setZero();

  A.block(0, 3, 3, 3) = Eigen::Matrix3d::Identity();
  A.block(3, 3, 3, 3) = -1.0 * drag_coefficients_.asDiagonal();

  Eigen::MatrixXd B;
  B.resize(6, 3);
  B.setZero();

  B(3, 1) = kGravity;
  B(4, 0) = -1.0 * kGravity;
  B(5, 2) = 1.0;

  Eigen::MatrixXd G = B * R.inverse() * B.transpose();

  Eigen::MatrixXd z11 = A;
  Eigen::MatrixXd z12 = -1.0 * G;
  Eigen::MatrixXd z21 = -1.0 * Q;
  Eigen::MatrixXd z22 = -1.0 * A.transpose();

  Eigen::MatrixXd Z;
  Z.resize(z11.rows() + z21.rows(), z11.cols() + z12.cols());
  Z << z11, z12, z21, z22;

  int n = A.cols();
  Eigen::MatrixXd U(2 * n, 2 * n);  // Orthogonal matrix from Schur decomposition
  Eigen::VectorXd WR(2 * n);
  Eigen::VectorXd WI(2 * n);
  lapack_int sdim = 0;  // Number of eigenvalues for which sort is true
  lapack_int info;
  info = LAPACKE_dgees(LAPACK_COL_MAJOR,  // Eigen default storage order
      'V',               // Schur vectors are computed
      'S',               // Eigenvalues are sorted
      select_lhp,        // Ordering callback
      Z.rows(),          // Dimension of test matrix
      Z.data(),          // Pointer to first element
      Z.rows(),          // Leading dimension (column stride)
      &sdim,             // Number of eigenvalues sort is true
      WR.data(),         // Real portion of eigenvalues
      WI.data(),         // Complex portion of eigenvalues
      U.data(),          // Orthogonal transformation matrix
      Z.rows());         // Dimension of Z

  Eigen::MatrixXd U11 = U.block(0, 0, n, n).transpose();
  Eigen::MatrixXd U21 = U.block(n, 0, n, n).transpose();

  return U11.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(U21).transpose();
}

bool NonlinearModelPredictiveControl::getCurrentReference(
    mav_msgs::EigenTrajectoryPoint* reference) const
{
  assert(reference != nullptr);

  (*reference).position_W = position_ref_.front();
  (*reference).velocity_W = velocity_ref_.front();
  (*reference).acceleration_W = acceleration_ref_.front();
  (*reference).setFromYaw(yaw_ref_.front());

  return true;
}

bool NonlinearModelPredictiveControl::getCurrentReference(
    mav_msgs::EigenTrajectoryPointDeque* reference) const
{
  assert(reference != nullptr);

  (*reference).clear();

  for (size_t i = 0; i < position_ref_.size(); i++) {
    mav_msgs::EigenTrajectoryPoint pnt;
    pnt.position_W = position_ref_.at(i);
    pnt.velocity_W = velocity_ref_.at(i);
    pnt.acceleration_W = acceleration_ref_.at(i);
    pnt.setFromYaw(yaw_ref_.at(i));
    (*reference).push_back(pnt);
  }
  return true;
}

bool NonlinearModelPredictiveControl::getPredictedState(
    mav_msgs::EigenTrajectoryPointDeque* predicted_state) const
{
  assert(predicted_state != nullptr);

  for (size_t i = 0; i < ACADO_N + 1; i++) {
    mav_msgs::EigenTrajectoryPoint pnt;
    pnt.position_W = state_.block(i, 6, 1, 3).transpose();
    (*predicted_state).push_back(pnt);
  }

  return true;
}

}
