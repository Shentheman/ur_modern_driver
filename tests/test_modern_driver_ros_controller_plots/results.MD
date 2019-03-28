# `ur_modern_driver` - ROS controllers
* This is not available in [`universal_robot`](https://github.com/ros-industrial/universal_robot)
* Run the example
  * `roslaunch ur_modern_driver ur10_ros_control.launch robot_ip:=IP`
  * `rosrun despot_yuri test_modern_driver_ros_controller.py`
* Based on `pos_control_time_gap_send_cmd_again_0.3.pdf`, `pos_control_time_gap_send_cmd_again_-1.pdf`, `vel_control_time_gap_send_cmd_again_0.3.pdf` and `vel_control_time_gap_send_cmd_again_-1.pdf`, the position and velocity based ROS controller seems to be working very well
  * We don't need to send time-optimal trajectories. We just need to send the goal as a single waypoint. The controller will find a trajectory to move the robot current position and velocity to goal position and velocity
  * We can also keep sending new goals to the robot
    * The good thing about the ROS controller is that the controller does not need to brake the robot and bring the velocity to `0` before executing a new goal, as the regular controller in `ur_modern_driver` and `universal_robot` do
