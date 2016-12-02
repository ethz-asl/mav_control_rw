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

#include "mav_control_interface/mpc_queue.h"

namespace mav_control {

MPCQueue::MPCQueue(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh, int mpc_queue_size)
    : nh_(nh),
      private_nh_(private_nh),
      mpc_queue_size_(mpc_queue_size),
      maximum_queue_size_(10000),
      current_queue_size_(0),
      initialized_(false),
      prediction_sampling_time_(0.01),
      queue_dt_(0.01),
      queue_start_time_(0.0)
{
  trajectory_reference_vis_publisher_ = nh_.advertise<visualization_msgs::Marker>( "reference_trajectory", 0 );
  //publish at 10 hz
  publish_queue_marker_timer_ = nh_.createTimer(ros::Duration(0.1), &MPCQueue::publishQueueMarker, this);
  private_nh_.param<std::string>("reference_frame", reference_frame_id_, "odom");
  minimum_queue_size_ = mpc_queue_size_*std::ceil(prediction_sampling_time_/queue_dt_);
}

MPCQueue::~MPCQueue()
{

}

void MPCQueue::clearQueue()
{
  position_reference_.clear();
  velocity_reference_.clear();
  acceleration_reference_.clear();
  yaw_reference_.clear();
  queue_start_time_ = 0.0;

  current_queue_size_ = 0;
}

void MPCQueue::initializeQueue(const mav_msgs::EigenTrajectoryPoint& point,
		double controller_sampling_time, double prediction_sampling_time) {
	if (initialized_)
		return;

	queue_dt_ = controller_sampling_time;
	prediction_sampling_time_ = prediction_sampling_time;
	minimum_queue_size_ = mpc_queue_size_
			* std::ceil(prediction_sampling_time_ / queue_dt_);

	clearQueue();

	fillQueueWithPoint(point);

	initialized_ = true;
}

void MPCQueue::initializeQueue(const mav_msgs::EigenOdometry& odometry,
		double controller_sampling_time, double prediction_sampling_time) {
	if (initialized_)
		return;

	queue_dt_ = controller_sampling_time;
	prediction_sampling_time_ = prediction_sampling_time;
	minimum_queue_size_ = mpc_queue_size_
			* std::ceil(prediction_sampling_time_ / queue_dt_);

	clearQueue();

	mav_msgs::EigenTrajectoryPoint point;

	point.position_W = odometry.position_W;
	point.velocity_W = odometry.getVelocityWorld();
	point.orientation_W_B = odometry.orientation_W_B;
	point.angular_velocity_W = odometry.angular_velocity_B;

	fillQueueWithPoint(point);

	initialized_ = true;
}

void MPCQueue::initializeQueue(double controller_sampling_time,
		double prediction_sampling_time) {
	if (initialized_)
		return;

	queue_dt_ = controller_sampling_time;
	prediction_sampling_time_ = prediction_sampling_time;
	minimum_queue_size_ = mpc_queue_size_
			* std::ceil(prediction_sampling_time_ / queue_dt_);

	clearQueue();
	mav_msgs::EigenTrajectoryPoint point;
	fillQueueWithPoint(point);

	initialized_ = true;
}

void MPCQueue::fillQueueWithPoint(const mav_msgs::EigenTrajectoryPoint& point)
{
  while (current_queue_size_ < minimum_queue_size_)
    pushBackPoint(point);
}

void MPCQueue::insertReference(const mav_msgs::EigenTrajectoryPoint& point)
{
  clearQueue();
  fillQueueWithPoint(point);
  queue_start_time_ = 0.0;
}

void MPCQueue::insertReferenceTrajectory(const mav_msgs::EigenTrajectoryPointDeque& queue)
{

  mav_msgs::EigenTrajectoryPointDeque interpolated_queue;
  linearInterpolateTrajectory(queue, interpolated_queue);

  {
    // Two options: if time is 0 or < than queue start time, shrink the queue to
    // minimum and insert.
    // If time is > queue start time, either insert at the correct position or append.
    double commanded_time_from_start = static_cast<double>(
        interpolated_queue.begin()->time_from_start_ns) * 1e-9;

    //ROS_WARN("Commanded time from start: %f, queue time: %f", commanded_time_from_start, queue_start_time_);

    if (commanded_time_from_start <= queue_start_time_ || commanded_time_from_start <= 1e-4) {
      clearQueue();
      queue_start_time_ = commanded_time_from_start;
    } else {
      // Find where this insertion point belongs and clear the queue out up to that point.
      double queue_end_time = queue_start_time_ + current_queue_size_ * queue_dt_;
      if (commanded_time_from_start < queue_end_time) {
        size_t start_index = std::round((commanded_time_from_start - queue_start_time_)/queue_dt_);

        double last_ref = (position_reference_.begin() + start_index)->x();
        position_reference_.erase(position_reference_.begin() + start_index, position_reference_.end());
        velocity_reference_.erase(velocity_reference_.begin() + start_index, velocity_reference_.end());
        acceleration_reference_.erase(acceleration_reference_.begin() + start_index, acceleration_reference_.end());
        yaw_reference_.erase(yaw_reference_.begin() + start_index, yaw_reference_.end());
        yaw_rate_reference_.erase(yaw_rate_reference_.begin() + start_index, yaw_rate_reference_.end());
        current_queue_size_ = start_index; // +/- 1?

        //ROS_INFO("Queue start time: %f, queue end time: %f, reference start time: %f, start_index: %d", queue_start_time_, queue_end_time, commanded_time_from_start, start_index);
        //ROS_INFO("Position x at queue start index: %f -> [...%f %f...], position x at command start: %f",
        //    last_ref, (position_reference_.end()-2)->x(), position_reference_.back().x(), interpolated_queue.front().position_W.x());
      }
      // Else just append to the end as before.
    }

    for (auto it = interpolated_queue.begin(); it != interpolated_queue.end(); ++it) {
      position_reference_.push_back(it->position_W);
      velocity_reference_.push_back(it->velocity_W);
      acceleration_reference_.push_back(it->acceleration_W);
      yaw_reference_.push_back(it->getYaw());
      yaw_rate_reference_.push_back(it->getYawRate());
      current_queue_size_++;
    }
  }

  if (current_queue_size_ < minimum_queue_size_) {
    fillQueueWithPoint(interpolated_queue.back());
  }

  //ROS_INFO("Current queue size: %d, current actual size: %d", current_queue_size_, position_reference_.size());
}

void MPCQueue::pushBackPoint(const mav_msgs::EigenTrajectoryPoint& point)
{
  if (current_queue_size_ < maximum_queue_size_) {
    position_reference_.push_back(point.position_W);
    velocity_reference_.push_back(point.velocity_W);
    acceleration_reference_.push_back(point.acceleration_W);
    yaw_reference_.push_back(point.getYaw());
    yaw_rate_reference_.push_back(point.getYawRate());
    current_queue_size_++;
  } else {
    ROS_WARN_STREAM_THROTTLE(1, "MPC: maximum queue size reached, discarding last reference point");
  }
}

void MPCQueue::shrinkQueueToMinimum()
{
  while (current_queue_size_ > minimum_queue_size_) {
    popBackPoint();
  }
}

void MPCQueue::popFrontPoint()
{
  if (current_queue_size_ > 0) {
    position_reference_.pop_front();
    velocity_reference_.pop_front();
    acceleration_reference_.pop_front();
    yaw_reference_.pop_front();
    yaw_rate_reference_.pop_front();
    queue_start_time_ += queue_dt_;
    current_queue_size_--;
  }
}

void MPCQueue::popBackPoint()
{
  if (current_queue_size_ > 0) {
    position_reference_.pop_back();
    velocity_reference_.pop_back();
    acceleration_reference_.pop_back();
    yaw_reference_.pop_back();
    yaw_rate_reference_.pop_back();
    current_queue_size_--;
  }
}

void MPCQueue::getLastPoint(mav_msgs::EigenTrajectoryPoint* point)
{
  assert(point!=NULL);
  if (current_queue_size_ > 0) {
    (*point).position_W = position_reference_.back();
    (*point).velocity_W = velocity_reference_.back();
    (*point).acceleration_W = acceleration_reference_.back();
    (*point).setFromYaw(yaw_reference_.back());
    (*point).setFromYawRate(yaw_rate_reference_.back());
  }
}

void MPCQueue::updateQueue()
{
  if (initialized_) {
    popFrontPoint();

    mav_msgs::EigenTrajectoryPoint point;
    getLastPoint(&point);

    while (current_queue_size_ < minimum_queue_size_)
      pushBackPoint(point);
  }
}

void MPCQueue::publishQueueMarker(const ros::TimerEvent&)
{
  if (trajectory_reference_vis_publisher_.getNumSubscribers() > 0) {
    visualization_msgs::Marker marker_queue;
    marker_queue.header.frame_id = reference_frame_id_;
    marker_queue.header.stamp = ros::Time();
    marker_queue.type = visualization_msgs::Marker::LINE_STRIP;
    marker_queue.scale.x = 0.05;
    marker_queue.color.a = 1.0;
    marker_queue.color.g = 1.0;

//    marker_heading.type = visualization_msgs::Marker::ARROW;
    {
      for (size_t i = 0; i < current_queue_size_; i++) {
        geometry_msgs::Point p;
        p.x = position_reference_.at(i).x();
        p.y = position_reference_.at(i).y();
        p.z = position_reference_.at(i).z();
        marker_queue.points.push_back(p);
      }
    }

    trajectory_reference_vis_publisher_.publish(marker_queue);
  }
}

void MPCQueue::printQueue()
{
  for (size_t i = 0; i < current_queue_size_; i++) {
    std::cout << "pos[" << i << "]\t" << position_reference_.at(i).transpose() << std::endl;
  }
}

void MPCQueue::getQueue(Vector3dDeque& position_reference, Vector3dDeque& velocity_reference,
                        Vector3dDeque& acceleration_reference, std::deque<double>& yaw_reference,
                        std::deque<double>& yaw_rate_reference)
{
	position_reference.clear();
	velocity_reference.clear();
	acceleration_reference.clear();
	yaw_reference.clear();
	yaw_rate_reference.clear();

  int N = std::ceil(prediction_sampling_time_/queue_dt_);
  for(int i=0; i<N*std::floor(current_queue_size_/N); i=i+N){
	  position_reference.push_back(position_reference_.at(i));
	  velocity_reference.push_back(velocity_reference_.at(i));
	  acceleration_reference.push_back(acceleration_reference_.at(i));
	  yaw_reference.push_back(yaw_reference_.at(i));
	  yaw_rate_reference.push_back(yaw_rate_reference_.at(i));
  }
}

void MPCQueue::linearInterpolateTrajectory(const mav_msgs::EigenTrajectoryPointDeque& input_queue,
                                           mav_msgs::EigenTrajectoryPointDeque& interpolated_queue)
{
  if(input_queue.size() < 2){
    interpolated_queue = input_queue;
    ROS_WARN_THROTTLE(0.1,"MPCQueue: Cannot interpolate reference queue, queue size < 2");
    return;
  }

  std::vector<int64_t> time_input;
  std::vector<int64_t> time_output;

  //fill time_input vector from input_queue
  // This is a default DT if none is specified. Corresponds to 100 Hz.
  const int64_t kDefaultDtNsec = 10000000;
  int64_t time_prev = 0;
  for (auto it = input_queue.begin(); it != input_queue.end(); ++it) {
    int64_t current_time = it->time_from_start_ns;
    // Check if time is uinitialized!
    if (it != input_queue.begin() && current_time == 0) {
      current_time = time_prev + kDefaultDtNsec;
      time_prev = current_time;
    }
    time_input.push_back(current_time);
  }

  int64_t time_0 = time_input.front();
  int64_t queue_dt_ns = (int64_t) (queue_dt_ * 1.0e9);
//
  time_output.push_back(time_0);  //set t0;
//
  int N = (time_input.back() - time_input.front()) / queue_dt_ns + 1;
//  std::cout << "N: " << N << std::endl;
//
//  //fill time_output vector
  for (size_t i = 1; i < N; i++)
    time_output.push_back(time_0 + queue_dt_ns * i);

  //for each time_output find the first larger element in the time_input vector
  for (auto it = time_output.begin(); it != time_output.end(); ++it) {
    mav_msgs::EigenTrajectoryPoint point;

    std::vector<int64_t>::iterator sol = std::upper_bound(time_input.begin(), time_input.end(),
                                                        *it);
    if(sol == time_input.end()){
      sol--;
    }
    int64_t time1 = *(sol - 1);
    int64_t time2 = *sol;

    Eigen::Vector3d position_2 = input_queue.at(sol - time_input.begin()).position_W;
    Eigen::Vector3d position_1 = input_queue.at(sol - time_input.begin() - 1).position_W;

    point.position_W = position_1 + ((position_2 - position_1) / (time2 - time1)) * (*it - time1);

    Eigen::Vector3d velocity_2 = input_queue.at(sol - time_input.begin()).velocity_W;
    Eigen::Vector3d velocity_1 = input_queue.at(sol - time_input.begin() - 1).velocity_W;

    point.velocity_W = velocity_1 + ((velocity_2 - velocity_1) / (time2 - time1)) * (*it - time1);

    Eigen::Vector3d acceleration_2 = input_queue.at(sol - time_input.begin()).acceleration_W;
    Eigen::Vector3d acceleration_1 = input_queue.at(sol - time_input.begin() - 1).acceleration_W;

    point.acceleration_W = acceleration_1
        + ((acceleration_2 - acceleration_1) / (time2 - time1)) * (*it - time1);

    double yaw_2 = input_queue.at(sol - time_input.begin()).getYaw();
    double yaw_1 = input_queue.at(sol - time_input.begin() - 1).getYaw();

    point.setFromYaw(yaw_1 + ((yaw_2 - yaw_1) / (time2 - time1)) * (*it - time1));

    double yaw_rate_2 = input_queue.at(sol - time_input.begin()).getYawRate();
    double yaw_rate_1 = input_queue.at(sol - time_input.begin() - 1).getYawRate();

    point.setFromYawRate(
        yaw_rate_1 + ((yaw_rate_2 - yaw_rate_1) / (time2 - time1)) * (*it - time1));

    point.time_from_start_ns = *it;
    interpolated_queue.push_back(point);
  }
}

}  // namespace mav_control
