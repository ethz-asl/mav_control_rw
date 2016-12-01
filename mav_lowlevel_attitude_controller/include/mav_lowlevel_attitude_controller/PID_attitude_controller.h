/*
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 *
 * This code can be used only for academic, non-commercial use.
 * This code cannot be redistributed under any license, open source or otherwise.
 *
 * CVXGEN license: http://cvxgen.com/docs/license.html
 * FORCES license: http://forces.ethz.ch
 *
 */

#include <mav_msgs/conversions.h>
#include <mav_msgs/eigen_mav_msgs.h>

#include <ros/ros.h>

namespace mav_control {
class PIDAttitudeController
{
 public:
  PIDAttitudeController(const ros::NodeHandle& nh, const ros::NodeHandle private_nh);
  ~PIDAttitudeController();
  void InitializeParams();

  void SetPIDParameters(double roll_gain, double pitch_gain, double p_gain, double q_gain,
                        double r_gain, double roll_integrator_gain, double pitch_integrator_gain)
  {
    roll_gain_ = roll_gain;
    pitch_gain_ = pitch_gain;
    p_gain_ = p_gain;
    q_gain_ = q_gain;
    r_gain_ = r_gain;
    roll_integrator_gain_ = roll_integrator_gain;
    pitch_integrator_gain_ = pitch_integrator_gain;
  }

  void SetDesiredAttitude(double desired_roll, double desired_pitch, double desired_yaw_rate,
                          double desired_thrust)
  {
    attitude_thrust_reference_ << desired_roll, desired_pitch, desired_yaw_rate, desired_thrust;
  }

  void SetOdometry(const mav_msgs::EigenOdometry& odometry);

  void CalculateRotorVelocities(Eigen::VectorXd* rotor_velocities);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 private:
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  int n_rotors_;
  bool initialized_params_;

  Eigen::MatrixX4d angular_acc_to_rotor_velocities_;
  Eigen::Matrix3d inertia_;
  double yaw_rate_gain_;

  double roll_gain_;
  double pitch_gain_;

  double roll_integrator_gain_;
  double pitch_integrator_gain_;
  double max_integrator_error_;

  double p_gain_;
  double q_gain_;
  double r_gain_;

  double roll_error_integration_;
  double pitch_error_integration_;

  Eigen::Vector4d attitude_thrust_reference_;
  mav_msgs::EigenOdometry odometry_;

  void ComputeDesiredAngularAcc(Eigen::Vector3d* angular_acc);
};
}
