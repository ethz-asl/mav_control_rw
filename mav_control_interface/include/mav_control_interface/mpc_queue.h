/*
 * Copyright (c) 2015, Mina Kamel, ASL, ETH Zurich, Switzerland
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


#ifndef MAV_CONTROL_COMMON_MPC_QUEUE_H_
#define MAV_CONTROL_COMMON_MPC_QUEUE_H_

#include <mav_msgs/conversions.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <deque>
#include <chrono>
#include <thread>
#include <mutex>
#include <memory>
#include <visualization_msgs/Marker.h>

namespace mav_control {

typedef std::deque<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > Vector3dDeque;

class MPCQueue
{
 public:
  MPCQueue(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh, int mpc_queue_size);
  ~MPCQueue();

	void initializeQueue(const mav_msgs::EigenTrajectoryPoint& point,
			double controller_sampling_time, double prediction_sampling_time);

	void initializeQueue(const mav_msgs::EigenOdometry& odometry,
			double controller_sampling_time, double prediction_sampling_time);

	void initializeQueue(double controller_sampling_time,
			double prediction_sampling_time);

  void insertReference(const mav_msgs::EigenTrajectoryPoint& point);

  void insertReferenceTrajectory(const mav_msgs::EigenTrajectoryPointDeque& queue);

  void getQueue(Vector3dDeque& position_reference_, Vector3dDeque& velocity_reference_,
                Vector3dDeque& acceleration_reference, std::deque<double>& yaw_reference_,
                std::deque<double>& yaw_rate_reference_);

  void updateQueue();

  bool empty() const { return current_queue_size_ == 0; }

 private:

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  int minimum_queue_size_;
  int mpc_queue_size_;
  const int maximum_queue_size_;
  int current_queue_size_;
  bool initialized_;

  double prediction_sampling_time_;
  double queue_dt_;

  Vector3dDeque position_reference_;
  Vector3dDeque velocity_reference_;
  Vector3dDeque acceleration_reference_;
  std::deque<double> yaw_reference_;
  std::deque<double> yaw_rate_reference_;
  double queue_start_time_;

  void clearQueue();
  void fillQueueWithPoint(const mav_msgs::EigenTrajectoryPoint& point);
  void pushBackPoint(const mav_msgs::EigenTrajectoryPoint& point);
  void popFrontPoint();
  void popBackPoint();
  void getLastPoint(mav_msgs::EigenTrajectoryPoint* point);
  void shrinkQueueToMinimum();

  //interpolate the reference queue to the controller update rate
  void linearInterpolateTrajectory(const mav_msgs::EigenTrajectoryPointDeque& input_queue,  mav_msgs::EigenTrajectoryPointDeque& output_queue);


  void printQueue();

  ros::Publisher trajectory_reference_vis_publisher_;
  void publishQueueMarker(const ros::TimerEvent&);
  ros::Timer publish_queue_marker_timer_;
  std::string reference_frame_id_;
};

}  // namespace mav_control

#endif  // MAV_CONTROL_COMMON_MPC_QUEUE_H_
