/*
* Copyright (c) 2015, Markus Achtelik, ASL, ETH Zurich, Switzerland
* You can contact the author at <markus dot achtelik at mavt dot ethz dot ch>
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
* http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*/

#include <mav_control_interface/mav_control_interface.h>
#include <mav_control_interface/rc_interface_aci.h>

using namespace mav_control_interface;

class DummyController : public PositionControllerInterface{
 public:
  DummyController():PositionControllerInterface(){
  }

  virtual std::string getName() const {
    return std::string("dummy_controller");
  }

  virtual bool getUseAttitudeQuaternionCommand() const {
    return false;
  }

  virtual double getMass() const {
    return 1.0;
  }

  virtual bool setReference(const mav_msgs::EigenTrajectoryPoint& reference){
    ROS_INFO_STREAM("got reference: position=" << reference.position_W.transpose() << " yaw=" << reference.getYaw());
    reference_array_.clear();
    reference_array_.push_back(reference);
    return true;
  }

  virtual bool setReferenceArray(const mav_msgs::EigenTrajectoryPointDeque& reference_array){
    ROS_INFO_STREAM("got reference array:");
    for(const auto& reference : reference_array)
      ROS_INFO_STREAM("    position=" << reference.position_W.transpose() << " yaw=" << reference.getYaw());
    reference_array_ = reference_array;
    return true;
  }

  virtual bool setOdometry(const mav_msgs::EigenOdometry& odometry){
    ROS_INFO_STREAM("got state: position=" << odometry.position_W.transpose() <<
                    " attitude=" << odometry.orientation_W_B.coeffs().transpose());
    return true;
  }

  virtual bool calculateRollPitchYawrateThrustCommand(
      mav_msgs::EigenRollPitchYawrateThrust* attitude_thrust_command){
    ROS_INFO("calculateRollPitchYawrateThrustCommand");
    return true;
  }

  virtual bool getCurrentReference(mav_msgs::EigenTrajectoryPoint* reference) const {
    assert(reference != nullptr);

    if(reference_array_.empty())
      return false;

    *reference = reference_array_.front();
    return true;
  }

  virtual bool getCurrentReference(mav_msgs::EigenTrajectoryPointDeque* reference) const {
    assert(reference != nullptr);

    if(reference_array_.empty())
      return false;

    *reference = reference_array_;
    return true;
  }

  virtual bool getPredictedState(mav_msgs::EigenTrajectoryPointDeque* predicted_state) const
  {
    return false;
  }

  virtual bool calculateAttitudeThrustCommand(mav_msgs::EigenAttitudeThrust* attitude_thrust_command){
    ROS_INFO("calculateAttitudeThrustCommand");
    return true;
  }

 private:
  mav_msgs::EigenTrajectoryPointDeque reference_array_;

};


int main(int argc, char** argv){

  ros::init(argc, argv, "test_lowlevel_flightmanager");
  ros::NodeHandle nh, pnh("~");

  std::shared_ptr<RcInterfaceAci> rc(new RcInterfaceAci(nh));
  std::shared_ptr<DummyController> controller(new DummyController);

  MavControlInterface control_interface(nh, pnh, controller, rc);

  ros::spin();
}
