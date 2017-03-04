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

#include <ros/ros.h>
#include <mav_msgs/default_topics.h>
#include <mav_nonlinear_mpc/nonlinear_mpc_node.h>
#include <mav_control_interface/mav_control_interface.h>
#include <mav_control_interface/rc_interface_aci.h>

namespace mav_control {

NonLinearModelPredictiveControllerNode::NonLinearModelPredictiveControllerNode(
    const ros::NodeHandle& nh, const ros::NodeHandle& private_nh)
    : nonlinear_mpc_(nh, private_nh),
      controller_dyn_config_server_(private_nh)
{
  dynamic_reconfigure::Server<mav_nonlinear_mpc::NonLinearMPCConfig>::CallbackType f_controller;
  f_controller = boost::bind(&NonLinearModelPredictiveControllerNode::ControllerDynConfigCallback,
                             this, _1, _2);
  controller_dyn_config_server_.setCallback(f_controller);
}

NonLinearModelPredictiveControllerNode::~NonLinearModelPredictiveControllerNode()
{

}

bool NonLinearModelPredictiveControllerNode::setReferenceArray(
    const mav_msgs::EigenTrajectoryPointDeque& reference_array)
{
  nonlinear_mpc_.setCommandTrajectory(reference_array);
  return true;
}

bool NonLinearModelPredictiveControllerNode::setReference(
    const mav_msgs::EigenTrajectoryPoint& reference)
{
  nonlinear_mpc_.setCommandTrajectoryPoint(reference);
  return true;
}

void NonLinearModelPredictiveControllerNode::ControllerDynConfigCallback(
    mav_nonlinear_mpc::NonLinearMPCConfig &config, uint32_t level)
{
  Eigen::Vector3d q_position;
  Eigen::Vector3d q_velocity;
  Eigen::Vector2d q_attitude;

  Eigen::Vector3d r_command;
  Eigen::VectorXd control_limits(5);

  q_position << config.q_x, config.q_y, config.q_z;
  q_velocity << config.q_vx, config.q_vy, config.q_vz;
  q_attitude << config.q_roll, config.q_pitch;

  r_command << config.r_roll, config.r_pitch, config.r_thrust;

  control_limits << config.roll_max, config.pitch_max, config.yaw_rate_max, config.thrust_min, config
      .thrust_max;

  nonlinear_mpc_.setPositionPenality(q_position);
  nonlinear_mpc_.setVelocityPenality(q_velocity);
  nonlinear_mpc_.setAttitudePenality(q_attitude);
  nonlinear_mpc_.setCommandPenality(r_command);
  nonlinear_mpc_.setYawGain(config.K_yaw);
  nonlinear_mpc_.setControlLimits(control_limits);

  nonlinear_mpc_.setAltitudeIntratorGain(config.Ki_altitude);
  nonlinear_mpc_.setXYIntratorGain(config.Ki_xy);

  nonlinear_mpc_.setEnableIntegrator(config.enable_integrator);
  nonlinear_mpc_.setEnableOffsetFree(config.enable_offset_free);

  nonlinear_mpc_.applyParameters();

}

bool NonLinearModelPredictiveControllerNode::setOdometry(const mav_msgs::EigenOdometry& odometry)
{
  nonlinear_mpc_.setOdometry(odometry);
  return true;
}

bool NonLinearModelPredictiveControllerNode::calculateAttitudeThrustCommand(
    mav_msgs::EigenAttitudeThrust* attitude_thrust_command)
{
  ROS_WARN("calculateAttitudeThrustCommand not implemented");
  return false;
}

bool NonLinearModelPredictiveControllerNode::calculateRollPitchYawrateThrustCommand(mav_msgs::EigenRollPitchYawrateThrust* attitude_thrust_command){
  Eigen::Vector4d rpy_thrust;
  nonlinear_mpc_.calculateRollPitchYawrateThrustCommand(&rpy_thrust);
  attitude_thrust_command->roll = rpy_thrust(0);
  attitude_thrust_command->pitch = rpy_thrust(1);
  attitude_thrust_command->yaw_rate = rpy_thrust(2);
  attitude_thrust_command->thrust.z() = rpy_thrust(3);
  return true;
}

bool NonLinearModelPredictiveControllerNode::getCurrentReference(
    mav_msgs::EigenTrajectoryPoint* reference) const
{
  assert(reference != nullptr);
  return nonlinear_mpc_.getCurrentReference(reference);
}

bool NonLinearModelPredictiveControllerNode::getCurrentReference(
    mav_msgs::EigenTrajectoryPointDeque* reference) const
{
  assert(reference != nullptr);
  return nonlinear_mpc_.getCurrentReference(reference);
}

bool NonLinearModelPredictiveControllerNode::getPredictedState(
    mav_msgs::EigenTrajectoryPointDeque* predicted_state) const
{
  assert(predicted_state != nullptr);
  return nonlinear_mpc_.getPredictedState(predicted_state);
}

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "NonLinearModelPredictiveControllerNode");

  ros::NodeHandle nh, private_nh("~");

  std::shared_ptr<mav_control::NonLinearModelPredictiveControllerNode> mpc(
      new mav_control::NonLinearModelPredictiveControllerNode(nh, private_nh));

  std::shared_ptr<mav_control_interface::RcInterfaceBase> rc;
  AutopilotInterface::setupRCInterface(nh, &(rc.get()));

  mav_control_interface::MavControlInterface control_interface(nh, private_nh, mpc, rc);

  ros::spin();

  return 0;
}
