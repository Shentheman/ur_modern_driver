# `ur_modern_driver` - regular controller
* This should be the same as [`universal_robot`](https://github.com/ros-industrial/universal_robot)
* Run the example
  * `roslaunch ur_modern_driver ur10_bringup.launch robot_ip:=IP`
  * `rosrun despot_yuri test_modern_driver_regular_controller.py`
* If we set `time_gap_send_cmd_again=0.3`, we have `pos_vel0.3.pdf`
  * Note that there is a `170` ms time delay between the controller issuing the commands and the robot starting execution
  * If we keep sending goals in every `0.3` sec, every time the robot can only move a little distance. In the end, the robot cannot even reach the goal within `completion_time`
* Therefore, we have to increase `time_gap_send_cmd_again` to `0.6`
* After we set `time_gap_send_cmd_again=0.6`, we have `pos_vel0.6.pdf`
  * As we can see, within every `0.6` sec, we send a new trajectory to the controller. However, the controller has to brake and change its velocity back to `0`, before executing new trajectories
  * The brake is not perfect though. Since the controller has to use a big acceleration to bring the velocity back to `0`, the robot velocity actually overshoot every time to a positive velocity
* In conclusion, we cannot make the robot smooth if we have to keep sending new commands to the robot while the robot is still executing previous trajectories