#include <eigen_conversions/eigen_msg.h>
#include <mav_msgs/conversions.h>
#include <tf/transform_datatypes.h>

#include "mav_control_interface/autopilot_interface.h"

#include "mav_control_interface/rc_interface_aci.h"
#include "mav_control_interface/rc_interface_mavros.h"

const std::string AutopilotInterface::kDefaultAutopilotInterface = "asctec";

const double MavRosCommandPublisher::kDefaultYawGain = 1.0;

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
    command_pub_ptr_ = std::make_shared<AscTecCommandPublisher>(nh, private_nh);
  } else if (autopilot_type == AutopilotInterface::Types::MAVROS) {
#ifdef USING_MAVROS
    command_pub_ptr_ = std::make_shared<MavRosCommandPublisher>(nh, private_nh);
#else
    ROS_FATAL(
        "THIS NODE HAS BEEN COMPILED WITHOUT MAVROS, CANNOT USE MAVROS "
        "INTERFACE");
#endif
  }
}

void CommandInterface::publishCommand(
    const mav_msgs::EigenRollPitchYawrateThrust& command, double thrust_min,
    double thrust_max) {
  if (command_pub_ptr_ != nullptr) {
    command_pub_ptr_->publishCommand(command, thrust_min, thrust_max);
  } else {
    ROS_FATAL(
        "Autopilot interface has not been set up, could not publish "
        "command");
  }
}

BaseCommandPublisher::BaseCommandPublisher(const ros::NodeHandle& nh,
                                           const ros::NodeHandle& private_nh)
    : nh_(nh), private_nh_(private_nh) {}

AscTecCommandPublisher::AscTecCommandPublisher(
    const ros::NodeHandle& nh, const ros::NodeHandle& private_nh)
    : BaseCommandPublisher(nh, private_nh) {
  attitude_and_thrust_command_publisher_ =
      nh_.advertise<mav_msgs::RollPitchYawrateThrust>(
          mav_msgs::default_topics::COMMAND_ROLL_PITCH_YAWRATE_THRUST, 1);
}

void AscTecCommandPublisher::publishCommand(
    const mav_msgs::EigenRollPitchYawrateThrust& command, double thrust_min,
    double thrust_max) {
  mav_msgs::RollPitchYawrateThrustPtr msg(new mav_msgs::RollPitchYawrateThrust);
  mav_msgs::EigenRollPitchYawrateThrust tmp_command = command;
  tmp_command.thrust.x() = 0;
  tmp_command.thrust.y() = 0;
  tmp_command.thrust.z() = std::max(0.0, command.thrust.z());

  msg->header.stamp = ros::Time::now();  // TODO(acmarkus): get from msg
  mav_msgs::msgRollPitchYawrateThrustFromEigen(command, msg.get());
  attitude_and_thrust_command_publisher_.publish(msg);
}

MavRosCommandPublisher::MavRosCommandPublisher(
    const ros::NodeHandle& nh, const ros::NodeHandle& private_nh)
    : BaseCommandPublisher(nh, private_nh) {
  command_publisher_ = nh_.advertise<mavros_msgs::AttitudeTarget>(
      "mavros/setpoint_raw/attitude", 1);

  // not actually just imu, mavros fills in orientation from fcu
  orientation_subscriber_ =
      nh_.subscribe("mavros/imu/data", 10,
                    &MavRosCommandPublisher::orientationCallback, this);

  private_nh_.param("yaw_gain", yaw_gain_, kDefaultYawGain);
}

void MavRosCommandPublisher::orientationCallback(
    const sensor_msgs::ImuConstPtr& msg) {
  Eigen::Quaterniond orientation;
  tf::quaternionMsgToEigen(msg->orientation, orientation);
  internal_yaw_ = mav_msgs::yawFromQuaternion(orientation);
}

void MavRosCommandPublisher::publishCommand(
    const mav_msgs::EigenRollPitchYawrateThrust& command, double thrust_min,
    double thrust_max) {
  mavros_msgs::AttitudeTarget command_msg;
  command_msg.header.stamp = ros::Time::now();
  command_msg.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE |
                          mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE |
                          mavros_msgs::AttitudeTarget::IGNORE_YAW_RATE;

  // Horrible hacks here, fudging a yaw setpoint from the commanded rate
  // A better solution is needed, but this will get us flying

  geometry_msgs::Quaternion attitudetarget_orientation_msg;
  tf::Quaternion attitudetarget_orientation;
  attitudetarget_orientation.setRPY(command.roll, command.pitch, internal_yaw_ + yaw_gain_*command.yaw_rate);
  attitudetarget_orientation_msg.x = attitudetarget_orientation.x();
  attitudetarget_orientation_msg.y = attitudetarget_orientation.y();
  attitudetarget_orientation_msg.z = attitudetarget_orientation.z();
  attitudetarget_orientation_msg.w = attitudetarget_orientation.w();
  geometry_msgs::Vector3 attitudetarget_bodyrate_msg;
  attitudetarget_bodyrate_msg.x = 0.0;
  attitudetarget_bodyrate_msg.y = 0.0;
  attitudetarget_bodyrate_msg.z = 0.0;
  command_msg.orientation = attitudetarget_orientation_msg;
  command_msg.body_rate = attitudetarget_bodyrate_msg;

  
 // command_msg.orientation = tf::createQuaternionMsgFromRollPitchYaw(
  //    command.roll, command.pitch,
   //   internal_yaw_);

  std::cout << "command.roll: " << command.roll << "\t command pitch: "<< command.pitch << std::endl;

  // throttle must be between 0 and 1 (use min and max thrust to get there)
  double thrust = (command.thrust.z()/2.27)/20.5; // scaling from data fitting
  //    (command.thrust.z() - thrust_min) / (thrust_max - thrust_min);

  command_msg.thrust = std::min(1.0, std::max(0.0, thrust));
  command_publisher_.publish(command_msg);

  return;
}
