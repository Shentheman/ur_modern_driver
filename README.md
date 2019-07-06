# Controller options
## Universal Robots Controller
* `roslaunch ur_modern_driver ur10_bringup.launch robot_ip:=IP`
  * Or `roslaunch ur_bringup ur10_bringup.launch robot_ip:=IP`
  * Implemented by Universal Robots
  * Available in both [`ur_modern_driver`](https://github.com/ros-industrial/ur_modern_driver) and [`universal_robot`](https://github.com/ros-industrial/universal_robot)
* These controllers provide a ROS action interface - `/follow_joint_trajectory`
* Cons
  * There is no smooth transition between the old and new trajectories
    * Based on our tests in `/tests/test_modern_driver_regular_controller_plots`, these controllers does not support smooth `trajectory replacement`. When a new trajectory is sent to the controller, the controller will have to stop the previous execution by braking the robot, before executing the new trajectory
    * This means that we cannot keep sending the controller updated new trajectories

## ROS Controllers
* `roslaunch ur_modern_driver ur10_ros_control.launch robot_ip:=IP`
  * Implemented by [ROS](http://wiki.ros.org/ros_controllers?distro=melodic)
  * [`ur_modern_driver`](https://github.com/ros-industrial/ur_modern_driver) bridges Universal Robots drivers with ROS controllers
* Pros
  * ROS controllers support smooth [`trajectory replacement`](http://wiki.ros.org/joint_trajectory_controller/UnderstandingTrajectoryReplacement)
    * https://github.com/ros-industrial/ur_modern_driver/issues/66
    * https://github.com/ros-industrial/ur_modern_driver/issues/36
    * This means that we can keep sending the controller updated trajectories and the controller will make the transition smooth
* There are several types of ROS controllers provided by [`ur_modern_driver`](https://github.com/ros-industrial/ur_modern_driver)
  * `vel_based_pos_traj_controller`
    * Fast to execute
      * Trajectory execution start within `50-70` ms
  * `pos_based_pos_traj_controller`
    * Slow to execute
      * Trajectory execution start within `150-180` ms
      * `Universal Robots Controller` is also in the `170` ms range
    * `pos_based_pos_traj_controller` can make robot stay closer to the commanded path than `vel_based_pos_traj_controller`
    * **ISSUE 1**
      * [the PID parameters for the `pos_based_pos_traj_controller` is not properly tweaked by `ur_modern_driver`](https://github.com/ros-industrial/ur_modern_driver)
      * You will see the robot jerky when executing trajectories via `pos_based_pos_traj_controller`
      * We have made it better (might still suboptimal) by
        * [`if I increase this value to 0.016 (i.e.), the position control runs as expected`](https://github.com/ros-industrial/ur_modern_driver/issues/132#issuecomment-326222790)
        * We add the argument `servoj_time` into [`ur10_ros_control_pos.launch`](https://github.com/Shentheman/ur_modern_driver/blob/6a821127ab4f046c7e6a20d9d1d7615c627f4330/launch/ur10_ros_control_pos.launch#L23)

    * **ISSUE 2**
      * Based on [`ur_modern_driver`](https://github.com/ros-industrial/ur_modern_driver), to run `pos_based_pos_traj_controller`, you should first launch `ur10_ros_control.launch` and then switch the `vel_based_pos_traj_controller` to `pos_based_pos_traj_controller`
      * However, sometimes when you execute trajectories via `pos_based_pos_traj_controller`, the robot does not move at all
      * This does not seem to be a problem of ROS controllers
      * One possible reason is that the switching controller does not actually affect the interface between the software and the hardware
      * To fix this issue, we split `ur10_ros_control.launch` into [`ur10_ros_control_pos.launch`](https://github.com/Shentheman/ur_modern_driver/blob/irg/launch/ur10_ros_control_pos.launch) and [`ur10_ros_control_vel.launch`](https://github.com/Shentheman/ur_modern_driver/blob/irg/launch/ur10_ros_control_vel.launch) to avoid the needs of switching controllers

  * The difference between the `/vel_based_pos_traj_controller` and the `/pos_based_pos_traj_controller` is in the type of commands that are sent to the robot
    * The velocity based controller sends joint speed commands to the robot, using the `speedj` command
    * The position based controller sends joint position commands to the robot, using the `servoj` command
      * **ISSUE** On the `Log` page on the teach pedant, you always see `speedj` even we are running `servoj`, although the robot motion looks fine
    * They both interpret the joint plan in exactly the same way
    * https://github.com/ros-industrial/ur_modern_driver/issues/155
* Our tests are available in `/tests/test_modern_driver_ros_controller_plots`

## `/joint_speed` interface
* Implemented by [`ur_modern_driver`](https://github.com/ros-industrial/ur_modern_driver)
* Takes messages of type `trajectory_msgs/JointTrajectory`. Parses the first JointTracetoryPoint and sends the specified joint speeds and accelerations to the robot.
* This interface is intended for doing visual servoing and other kind of control that requires speed control rather than position control of the robot.
* Our tests are available in `/tests/test_modern_driver_joint_speed_topic_plots`


# Run tests
* ROS `pos` Controller
  * `roslaunch ur_modern_driver ur10_ros_control_pos.launch robot_ip:=IP`
  * `rosrun ur_modern_driver test_modern_driver_ros_controller.py`
    * Configure `test_modern_driver_ros_controller.py` for `pos` control mode
* ROS `vel` Controller
  * `roslaunch ur_modern_driver ur10_ros_control_vel.launch robot_ip:=IP`
  * `rosrun ur_modern_driver test_modern_driver_ros_controller.py`
    * Configure `test_modern_driver_ros_controller.py` for `vel` control mode
* Regular Universal Robots Controller
  * `roslaunch ur_modern_driver ur10_bringup.launch robot_ip:=IP`
    * Or `roslaunch ur_bringup ur10_bringup.launch robot_ip:=IP`
  * `rosrun ur_modern_driver test_modern_driver_regular_controller.py`
* `/joint_speed` interface for visual servoing
  * `roslaunch ur_modern_driver ur10_bringup.launch robot_ip:=IP`
  * `rosrun ur_modern_driver test_modern_driver_joint_speed_topic.py`
