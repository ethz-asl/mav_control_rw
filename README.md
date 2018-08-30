mav_control_rw [![Build Status](https://travis-ci.org/ethz-asl/mav_control_rw.svg?branch=master)](https://travis-ci.org/ethz-asl/mav_control_rw)
======

Control strategies for rotary wing Micro Aerial Vehicles (MAVs) using ROS

Overview
------

This repository contains controllers for rotary wing MAVs. Currently we support the following controllers:
- *mav_linear_mpc* : Linear MPC for MAV trajectory tracking
- *mav_nonlinear_mpc* : Nonlinear MPC for MAV trajectory tracking
- *PID_attitude_control* : low level PID attitude controller 

Moreover, an external disturbance observer based on Kalman Filter is implemented to achieve offset-free tracking. 

If you use any of these controllers within your research, please cite one of the following references

```bibtex
@incollection{kamelmpc2016,
                author      = "Mina Kamel and Thomas Stastny and Kostas Alexis and Roland Siegwart",
                title       = "Model Predictive Control for Trajectory Tracking of Unmanned Aerial Vehicles Using Robot Operating System",
                editor      = "Anis Koubaa",
                booktitle   = "Robot Operating System (ROS) The Complete Reference, Volume 2",
                publisher   = "Springer",
                year = “2017”,
}
```

```bibtex
@ARTICLE{2016arXiv161109240K,
          author = {{Kamel}, M. and {Burri}, M. and {Siegwart}, R.},
          title = "{Linear vs Nonlinear MPC for Trajectory Tracking Applied to Rotary Wing Micro Aerial Vehicles}",
          journal = {ArXiv e-prints},
          archivePrefix = "arXiv",
          eprint = {1611.09240},
          primaryClass = "cs.RO",
          keywords = {Computer Science - Robotics},
          year = 2016,
          month = nov
}

```


Installation instructions
------

To run the controller with RotorS simulator (https://github.com/ethz-asl/rotors_simulator), follow these instructions:

* Install and initialize ROS indigo desktop full, additional ROS packages, catkin-tools:
  
```sh
  $ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
  $ wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
  $ sudo apt-get update
  $ sudo apt-get install ros-indigo-desktop-full ros-indigo-joy ros-indigo-octomap-ros python-wstool python-catkin-tools
  $ sudo rosdep init
  $ rosdep update
  $ source /opt/ros/indigo/setup.bash
```
* Initialize catkin workspace:
```sh
  $ mkdir -p ~/catkin_ws/src
  $ cd ~/catkin_ws
  $ catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
  $ catkin init  # initialize your catkin workspace
```
* Get the controllers and dependencies
```sh
  $ sudo apt-get install liblapacke-dev
  $ git clone https://github.com/catkin/catkin_simple.git
  $ git clone https://github.com/ethz-asl/rotors_simulator.git
  $ git clone https://github.com/ethz-asl/mav_comm.git
  $ git clone https://github.com/ethz-asl/eigen_catkin.git

  $ git clone https://github.com/ethz-asl/mav_control_rw.git
```
* Build the workspace  
```sh
  $ catkin build
```

* Run the simulator and the linear MPC. In seperate terminals run the following commands
  
```sh
  $ roslaunch rotors_gazebo mav.launch mav_name:=firefly
```
```sh
  $ roslaunch mav_linear_mpc mav_linear_mpc_sim.launch mav_name:=firefly
```
You can use `rqt` to publish commands to the controller.


To run the controller with the multi sensor fusion (MSF) framewok (https://github.com/ethz-asl/ethzasl_msf):
* Get msf
```sh
  $ git clone https://github.com/ethz-asl/ethzasl_msf.git
```

* Run the simulator, the linear MPC and MSF, in seperate terminals run the following commands
  
```sh
  $ roslaunch rotors_gazebo mav.launch mav_name:=firefly
```
```sh
  $ roslaunch mav_linear_mpc mav_linear_mpc_sim_msf.launch mav_name:=firefly
```

Don't forget to initialize MSF. 

Supported autopilots
------
### Asctec Research Platforms
This control will work as is with the ros interface to the now discontinued Asctec research platforms (Hummingbird, Pelican, Firefly and Neo). 

### Pixhawk
This controller requires some small modifications to the PX4 firmware to allow yaw rate inputs. A modified version of the firmware can be found [here](https://github.com/ethz-asl/ethzasl_mav_px4). The firmware is interfaced with through a [modified mavros node](https://github.com/ethz-asl/mavros).

### DJI
The controller can interface with DJI platforms through our [mav_dji_ros_interface](https://github.com/ethz-asl/mav_dji_ros_interface)

Published and subscribed topics
------

The linear and nonlinear MPC controllers publish and subscribe to the following topics:

- Published topics:
  - **`command/roll_pitch_yawrate_thrust`** of type `mav_msgs/RollPitchYawrateThrust`. This is the command to the low level controller. Angles are in `rad` and `thrust` is in `N`.
  - **`command/current_reference`** of type `trajectory_msgs/MultiDOFJointTrajectory`. This is the current reference.
  -  **`state_machine/state_info`** of type `std_msgs/String`. This is the current state of the state machine of mav_control_interface.
  -  **`predicted_state`** of type `visualization_msgs/Marker`. This is the predicted vehicle positions that can be used for visualization in `rviz`.
  -  **`reference_trajectory`** of type `visualization_msgs/Marker`. This is the reference trajectory that can be used for visualization in `rviz`.
  - **`KF_observer/observer_state`** of type `mav_disturbance_observer/ObserverState`. This is the disturbance observer state used for debugging purposes. It includes estimated external forces and torques.
  
- Subscribed topics:
  - **`command/pose`** of type `geometry_msgs/PoseStamped`. This is a reference set point.
  - **`command/trajectory`** of type `trajectory_msgs/MultiDOFJointTrajectory`. This is a desired trajectory reference that includes desired velocities and accelerations.
  - **`rc`** of type `sensor_msgs/Joy`. This is the remote control commands for teleoperation purposes. It also serves to abort mission anytime.
  - **`odometry`** of type `nav_msgs/Odometry`. This is the current state of the vehicle. The odometry msg includes pose and twist information.
  
  
The PID attitude controller publishes and subscribes to the following topics:
- Published topics:
  - **`command/motor_speed`** of type `mav_msgs/Actuators`. This is the commanded motor speed.

- Subscribed topics:
  - **`command/roll_pitch_yawrate_thrust`** of type `mav_msgs/RollPitchYawrateThrust`.
  - **`odometry`** of type `nav_msgs/Odometry`.
  
 
 
Parameters
------
A summary of the linear and nonlinear MPC parameters:

| Parameter             | Description                                                                     |
| --------------------  |:-------------------------------------------------------------------------------:| 
| `use_rc_teleop`       | enable RC teleoperation. Set to `false` in case of simulation.                  |
| `reference_frame`     | the name of the reference frame.                                                |
| `verbose`             | controller prints on screen debugging information and computation time          |
| `mass`                | vehicle mass                                                                    | 
| `roll_time_constant`  | time constant of roll first order model                                         |
| `pitch_time_constant` | time constant of pitch first order model                                        |
|`roll_gain`            | gain of roll first order model                                                  |
|`pitch_gain`           | gain of pitch first order model                                                 |
|`drag_coefficients`    | drag on `x,y,z` axes                                                            |
|`q_x, q_y, q_z`*       | penalty on position error                                                       |
|`q_vx, q_vy, q_vz`*    | penalty on velocity error                                                       |
|`q_roll, q_pitch`*     | penalty on attitude state                                                       |
|`r_roll, r_pitch, r_thtust`*| penalty on control input                                                    |
|`r_droll, r_dpitch, r_dthtust`*| penalty on delta control input (only Linear MPC)                        |
|`roll_max, pitch_max, yaw_rate_max`*| limits of control input                                             |
|`thrust_min, thrust_max`* | limit on thrust control input in `m/s^2`                                      |
|`K_yaw`*                  | yaw P loop gain                                                               |
|`Ki_xy, Ki_z`*            | integrator gains on `xy` and `z` axes respectively                            |
|`position_error_integration_limit` | limit of position error integration                                 |
|`antiwindup_ball`        | if the error is larger than this ball, no integral action is applied          |
|`enable_offset_free`*     | use estimated disturbances to achieve offset free tracking                    |
|`enable_integrator`*      | use error integration to achieve offset free tracking                         |
|`sampling_time`          | the controller sampling time (must be equal to the rate of `odometry` message |
|`prediction_sampling_time`| the prediction sampling time inside the controller                           |


\* Through dynamic reconfigure, it is possible to change these parameters.

--------


A summary of the PID attitude parameters:

| Parameter             | Description                                                                     |
| --------------------  |:-------------------------------------------------------------------------------:| 
| `inertia`             | vehicle inertia  `3x3` matrix                                                   |
|`allocation_matrix`    | control allocation matrix depending on the configuration of the rotors          |
|`n_rotors`             | number of rotors                                                                |
|`rotor_force_constant` | force constant of the rotor in `N/rad^2` such that `F_i =rotor_force_constant*rotor_velocity^2`                                         |
|`rotor_moment_constant`| rotor moment constant such that `M = rotor_moment_constant*F_i`                 |
|`arm_length`           | distance between rotor and vehicle center                                       |
|`roll_gain, pitch_gain`* | error proportional term                                                       |
|`p_gain, q_gain, r_gain`* | derivative gain                                                              |
|`roll_int_gain, pitch_int_gain`*| integrator gains                                                       |
|`max_integrator_error`     | saturation on the integrator                                                |

\* Through dynamic reconfigure, it is possible to change these parameters.

 --------

References
------

[1] Model Predictive Control for Trajectory Tracking of Unmanned Aerial Vehicles Using Robot Operating System. Mina Kamel, Thomas Stastny, Kostas Alexis and Roland Siegwart. Robot Operating System (ROS) The Complete Reference Volume 2. Springer 2017 (to appear)

[2] Linear vs Nonlinear MPC for Trajectory Tracking Applied to Rotary Wing Micro Aerial Vehicles. Mina Kamel, Michael Burri and Roland Siegwart. arXiv:1611.09240

--------

Contact
-------
Mina Kamel fmina(at)ethz.ch
