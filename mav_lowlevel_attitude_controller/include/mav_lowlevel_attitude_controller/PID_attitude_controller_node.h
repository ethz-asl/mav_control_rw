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

#include <stdio.h>
#include <memory.h>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <mav_msgs/RollPitchYawrateThrust.h>
#include <mav_msgs/Actuators.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Eigen>

#include <dynamic_reconfigure/server.h>
#include <mav_lowlevel_attitude_controller/PIDAttitudeConfig.h>
#include <mav_lowlevel_attitude_controller/PID_attitude_controller.h>

namespace mav_control {

class PIDAttitudeControllerNode
{
 public:
  PIDAttitudeControllerNode(const ros::NodeHandle& nh, const ros::NodeHandle private_nh);
  ~PIDAttitudeControllerNode();

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 private:
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  PIDAttitudeController PID_attitude_controller_;

  //publishers
  ros::Publisher motor_velocity_reference_pub_;

  // subscribers
  ros::Subscriber command_roll_pitch_yawrate_thrust_sub_;
  ros::Subscriber odometry_sub_;

  bool got_first_attitude_command_;

  void CommandRollPitchYawRateThrustCallback(
      const mav_msgs::RollPitchYawrateThrustConstPtr& roll_pitch_yawrate_thrust_reference);
  void OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg);

  void DynConfigCallback(mav_linear_mpc::PIDAttitudeConfig &config, uint32_t level);

  dynamic_reconfigure::Server<mav_linear_mpc::PIDAttitudeConfig> dyn_config_server_;

};

}
