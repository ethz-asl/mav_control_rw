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

#include <mav_msgs/default_topics.h>

#include <mav_control_interface/mav_control_interface.h>
#include <mav_control_interface/rc_interface_aci.h>

#include <mav_linear_mpc/linear_mpc_node.h>

namespace mav_control {

LinearModelPredictiveControllerNode::LinearModelPredictiveControllerNode(
    const ros::NodeHandle& nh, const ros::NodeHandle& private_nh)
    : linear_mpc_(nh, private_nh),
      dyn_config_server_(private_nh)
{
  dynamic_reconfigure::Server<mav_linear_mpc::LinearMPCConfig>::CallbackType f;
  f = boost::bind(&LinearModelPredictiveControllerNode::DynConfigCallback, this, _1, _2);
  dyn_config_server_.setCallback(f);
}

LinearModelPredictiveControllerNode::~LinearModelPredictiveControllerNode()
{

}

void LinearModelPredictiveControllerNode::DynConfigCallback(mav_linear_mpc::LinearMPCConfig &config,
                                                            uint32_t level)
{
  Eigen::Vector3d q_position;
  Eigen::Vector3d q_velocity;
  Eigen::Vector2d q_attitude;

  Eigen::Vector3d r_command;
  Eigen::Vector3d r_delta_command;
  Eigen::VectorXd control_limits(5);

  q_position << config.q_x, config.q_y, config.q_z;
  q_velocity << config.q_vx, config.q_vy, config.q_vz;
  q_attitude << config.q_roll, config.q_pitch;

  r_command << config.r_roll, config.r_pitch, config.r_thrust;
  r_delta_command << config.r_droll, config.r_dpitch, config.r_dthrust;

  control_limits << config.roll_max, config.pitch_max, config.yaw_rate_max, config.thrust_min, config.thrust_max;

  linear_mpc_.setPositionPenality(q_position);
  linear_mpc_.setVelocityPenality(q_velocity);
  linear_mpc_.setAttitudePenality(q_attitude);
  linear_mpc_.setCommandPenality(r_command);
  linear_mpc_.setDeltaCommandPenality(r_delta_command);
  linear_mpc_.setYawGain(config.K_yaw);
  linear_mpc_.setControlLimits(control_limits);

  linear_mpc_.setAltitudeIntratorGain(config.Ki_altitude);
  linear_mpc_.setXYIntratorGain(config.Ki_xy);

  linear_mpc_.setEnableIntegrator(config.enable_integrator);
  linear_mpc_.setEnableOffsetFree(config.enable_offset_free);

  linear_mpc_.applyParameters();
}

bool LinearModelPredictiveControllerNode::setReference(
    const mav_msgs::EigenTrajectoryPoint& reference)
{
  linear_mpc_.setCommandTrajectoryPoint(reference);
  return true;
}

bool LinearModelPredictiveControllerNode::setReferenceArray(
    const mav_msgs::EigenTrajectoryPointDeque& reference_array)
{
  linear_mpc_.setCommandTrajectory(reference_array);
  return true;
}

bool LinearModelPredictiveControllerNode::setOdometry(const mav_msgs::EigenOdometry& odometry)
{
  linear_mpc_.setOdometry(odometry);
  return true;
}

bool LinearModelPredictiveControllerNode::calculateRollPitchYawrateThrustCommand(
    mav_msgs::EigenRollPitchYawrateThrust* attitude_thrust_command)
{
  Eigen::Vector4d rpy_thrust;
  linear_mpc_.calculateRollPitchYawrateThrustCommand(&rpy_thrust);
  attitude_thrust_command->roll = rpy_thrust(0);
  attitude_thrust_command->pitch = rpy_thrust(1);
  attitude_thrust_command->yaw_rate = rpy_thrust(2);
  attitude_thrust_command->thrust.z() = rpy_thrust(3);
  return true;
}

bool LinearModelPredictiveControllerNode::calculateAttitudeThrustCommand(
    mav_msgs::EigenAttitudeThrust* attitude_thrust_command)
{
  ROS_WARN("calculateAttitudeThrustCommand not implemented");
  return false;
}

bool LinearModelPredictiveControllerNode::getCurrentReference(
    mav_msgs::EigenTrajectoryPoint* reference) const
{
  assert(reference != nullptr);
  return linear_mpc_.getCurrentReference(reference);
}

bool LinearModelPredictiveControllerNode::getCurrentReference(
    mav_msgs::EigenTrajectoryPointDeque* reference) const
{
  assert(reference != nullptr);
  return linear_mpc_.getCurrentReference(reference);
}

bool LinearModelPredictiveControllerNode::getPredictedState(
    mav_msgs::EigenTrajectoryPointDeque* predicted_state) const
{
  assert(predicted_state != nullptr);
  return linear_mpc_.getPredictedState(predicted_state);
}

}  // end namespace mav_control

int main(int argc, char** argv)
{
  ros::init(argc, argv, "LinearModelPredictiveControllerNode");

  ros::NodeHandle nh, private_nh("~");

  std::shared_ptr<mav_control::LinearModelPredictiveControllerNode> mpc(
      new mav_control::LinearModelPredictiveControllerNode(nh, private_nh));

  std::shared_ptr<mav_control_interface::RcInterfaceAci> rc(
      new mav_control_interface::RcInterfaceAci(nh));

  mav_control_interface::MavControlInterface control_interface(nh, private_nh, mpc, rc);

  ros::spin();

  return 0;
}
