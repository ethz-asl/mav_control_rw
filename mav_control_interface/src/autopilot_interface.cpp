#include <tf/transform_datatypes.h>

#include "mav_control_interface/autopilot_interface.h"

#include "mav_control_interface/rc_interface_aci.h"
#include "mav_control_interface/rc_interface_mavros.h"

const std::string AutopilotInterface::kDefaultAutopilotInterface = "asctec";

AutopilotInterface::Types AutopilotInterface::getInterfaceType(
    const ros::NodeHandle& nh_in) {
  ros::NodeHandle nh = nh_in;
  std::string autopilot_interface_str;
  nh.param("autopilot_interface", autopilot_interface_str,
           kDefaultAutopilotInterface);

  std::transform(autopilot_interface_str.begin(), autopilot_interface_str.end(),
                 autopilot_interface_str.begin(), ::tolower);

  if (autopilot_interface_str == "asctec") {
    return Types::ASCTEC;
  } else if (autopilot_interface_str == "mavros") {
    return Types::MAVROS;
  } else {
    ROS_FATAL_STREAM("Unrecognized autopilot interface: "
                     << autopilot_interface_str
                     << " .Valid options are 'asctec' and 'mavros'.");
    return Types::INVALID;
  }
}

void AutopilotInterface::setupRCInterface(
    const ros::NodeHandle& nh, const ros::NodeHandle& private_nh,
    std::shared_ptr<mav_control_interface::RcInterfaceBase>* rc_interface_ptr) {
  Types autopilot_type = getInterfaceType(private_nh);

  if (autopilot_type == AutopilotInterface::Types::ASCTEC) {
    *rc_interface_ptr =
        std::make_shared<mav_control_interface::RcInterfaceAci>(nh);
  } else if (autopilot_type == AutopilotInterface::Types::MAVROS) {
    *rc_interface_ptr =
        std::make_shared<mav_control_interface::RcInterfaceMavRos>(nh);
  }
}

CommandInterface::CommandInterface(const ros::NodeHandle& nh,
                                   const ros::NodeHandle& private_nh) {
  AutopilotInterface::Types autopilot_type =
      AutopilotInterface::getInterfaceType(private_nh);

  if (autopilot_type == AutopilotInterface::Types::ASCTEC) {
    command_pub_ptr_ = std::make_shared<AscTecCommandPublisher>(nh);
  } else if (autopilot_type == AutopilotInterface::Types::MAVROS) {
#ifdef USING_MAVROS
    command_pub_ptr_ = std::make_shared<MavRosCommandPublisher>(nh);
#else
    ROS_FATAL(
        "THIS NODE HAS BEEN COMPILED WITHOUT MAVROS, CANNOT USE MAVROS "
        "INTERFACE");
#endif
  }
}

void CommandInterface::publishCommand(
    const mav_msgs::EigenRollPitchYawrateThrust& command, double yaw,
    double thrust_min, double thrust_max) {
  if (command_pub_ptr_ != nullptr) {
    command_pub_ptr_->publishCommand(command, yaw, thrust_min, thrust_max);
  } else {
    ROS_FATAL(
        "Autopilot interface, has not been set up, could not publish "
        "command");
  }
}

BaseCommandPublisher::BaseCommandPublisher(const ros::NodeHandle& nh)
    : nh_(nh) {}

AscTecCommandPublisher::AscTecCommandPublisher(const ros::NodeHandle& nh)
    : BaseCommandPublisher(nh) {
  attitude_and_thrust_command_publisher_ =
      nh_.advertise<mav_msgs::RollPitchYawrateThrust>(
          mav_msgs::default_topics::COMMAND_ROLL_PITCH_YAWRATE_THRUST, 1);
}

void AscTecCommandPublisher::publishCommand(
    const mav_msgs::EigenRollPitchYawrateThrust& command, double yaw,
    double thrust_min, double thrust_max) {
  mav_msgs::RollPitchYawrateThrustPtr msg(new mav_msgs::RollPitchYawrateThrust);
  mav_msgs::EigenRollPitchYawrateThrust tmp_command = command;
  tmp_command.thrust.x() = 0;
  tmp_command.thrust.y() = 0;
  tmp_command.thrust.z() = std::max(0.0, command.thrust.z());

  msg->header.stamp = ros::Time::now();  // TODO(acmarkus): get from msg
  mav_msgs::msgRollPitchYawrateThrustFromEigen(command, msg.get());
  attitude_and_thrust_command_publisher_.publish(msg);
}

MavRosCommandPublisher::MavRosCommandPublisher(const ros::NodeHandle& nh)
    : BaseCommandPublisher(nh) {
  attitude_command_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>(
      "mavros/setpoint_attitude/attitude", 1);
  throttle_command_publisher_ = nh_.advertise<std_msgs::Float64>(
      "mavros/setpoint_attitude/att_throttle", 1);
}

void MavRosCommandPublisher::publishCommand(
    const mav_msgs::EigenRollPitchYawrateThrust& command, double yaw,
    double thrust_min, double thrust_max) {
  geometry_msgs::PoseStamped attitude_msg;
  attitude_msg.header.stamp = ros::Time::now();

  // we need yaw not yaw rate so for now I just grab the predicted yaw
  // TODO find less horrible hack
  attitude_msg.pose.orientation =
      tf::createQuaternionMsgFromRollPitchYaw(command.roll, command.pitch, yaw);
  attitude_command_publisher_.publish(attitude_msg);

  std_msgs::Float64 throttle_msg;
  // throttle must be between 0 and 1 (use min and max thrust to get there)
  throttle_msg.data =
      (command.thrust.z() - thrust_min) / (thrust_max - thrust_min);
  throttle_msg.data = std::min(1.0, std::max(0.0, throttle_msg.data));
  throttle_command_publisher_.publish(throttle_msg);
  return;
}
