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

#ifndef LINEAR_MPC_NODE_H
#define LINEAR_MPC_NODE_H

#include <boost/bind.hpp>
#include <Eigen/Eigen>
#include <stdio.h>

//ros
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <mav_control_interface/position_controller_interface.h>
#include <mav_linear_mpc/linear_mpc.h>

//dynamic reconfiguration
#include <dynamic_reconfigure/server.h>
#include <mav_linear_mpc/LinearMPCConfig.h>

namespace mav_control {

class LinearModelPredictiveControllerNode: public mav_control_interface::PositionControllerInterface
{
 public:
  LinearModelPredictiveControllerNode(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh);
  ~LinearModelPredictiveControllerNode();

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 private:
  LinearModelPredictiveController linear_mpc_;

  dynamic_reconfigure::Server<mav_linear_mpc::LinearMPCConfig> dyn_config_server_;

  void DynConfigCallback(mav_linear_mpc::LinearMPCConfig &config, uint32_t level);

  virtual std::string getName() const
  {
    return std::string("linear_model_predictive_controller");
  }

  virtual bool getUseAttitudeQuaternionCommand() const
  {
    return false;
  }

  virtual double getMass() const {
    return linear_mpc_.getMass();
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

};

}
#endif
