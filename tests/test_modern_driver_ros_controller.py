#!/usr/bin/python
from __future__ import division
from IPython import embed
import numpy as np, rospy, copy, math, std_msgs, argparse, os, yaml
import matplotlib
import matplotlib.pyplot as plt
from catkin.find_in_workspaces import find_in_workspaces
from sensor_msgs.msg import JointState
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from controller_manager_msgs.srv import SwitchController

# $ roslaunch ur_modern_driver ur10_ros_control_pos.launch robot_ip:=IP
# $ roslaunch ur_modern_driver ur10_ros_control_vel.launch robot_ip:=IP

home_pose = [
    -1.1754127787170481e-05, -1.570775145281954, 2.0338028612711512e-05,
    -1.570784429061514, -1.0939200221216083e-05, 1.4323545606882795e-05
]


class Robot(object):
    def __init__(self, vel_control):
        self.joint_names = [
            "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
            "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
        ]
        self.num_dofs = len(self.joint_names)
        self.cur_jtpos = [0.0] * self.num_dofs
        self.cur_jtvel = [0.0] * self.num_dofs
        self.collecting_data_starting_time = None
        self.times = []
        self.jtposs = []
        self.jtvels = []
        joint_states_subscriber = rospy.Subscriber('/joint_states', JointState,
                                                   self.cur_jtstate_callback)

        self.control_client = None
        self.vel_control = vel_control

        """
        # Switching controller does not seem to work well
        # when interfacing with the real robot.
        # Instead, we make 2 launch files for the pos and vel
        # ROS control modes:
        # https://github.com/Shentheman/ur_modern_driver/blob/irg/launch/ur10_ros_control_pos.launch
        # https://github.com/Shentheman/ur_modern_driver/blob/irg/launch/ur10_ros_control_vel.launch

        # By default the velocity based controller is started
        # https://github.com/ros-industrial/ur_modern_driver
        start_controllers = ['pos_based_pos_traj_controller']
        stop_controllers = ['vel_based_pos_traj_controller']
        if self.vel_control:
            start_controllers = ['vel_based_pos_traj_controller']
            stop_controllers = ['pos_based_pos_traj_controller']
        try:
            service_name = 'controller_manager/switch_controller'
            rospy.wait_for_service(service_name)
            # http://wiki.ros.org/controller_manager
            swith_controller = rospy.ServiceProxy(service_name,
                                                  SwitchController)
            resp = swith_controller(
                start_controllers=start_controllers,
                stop_controllers=stop_controllers,
                strictness=1)
            assert (resp.ok)
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e
            raise RuntimeError
        """

        if self.vel_control:
            self.control_client = actionlib.SimpleActionClient(
                '/vel_based_pos_traj_controller/follow_joint_trajectory',
                FollowJointTrajectoryAction)
            print "Vel control"
        else:
            self.control_client = actionlib.SimpleActionClient(
                '/pos_based_pos_traj_controller/follow_joint_trajectory',
                FollowJointTrajectoryAction)
        assert (self.control_client.wait_for_server())
        rospy.sleep(1)

        print "Move to", home_pose, "?"
        embed()

        completion_time = 3.0
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = home_pose
        point.velocities = [0.0] * self.num_dofs
        point.time_from_start = rospy.Time(completion_time)
        goal.trajectory.points.append(point)
        self.control_client.send_goal(goal)
        self.control_client.wait_for_result(
            rospy.Duration.from_sec(completion_time))

        print "Just reach", home_pose
        embed()

    def test_control(self, time_gap_send_cmd_again=-1, save=False):
        freq = 500
        rate = rospy.Rate(freq)

        goal_pose = copy.deepcopy(self.cur_jtpos)
        goal_pose[0] -= 0.2
        pos2 = list(goal_pose)
        vel2 = [0.0] * self.num_dofs

        completion_time = 1.0

        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = pos2
        point.velocities = vel2
        point.time_from_start = rospy.Time(completion_time)
        goal.trajectory.points.append(point)

        self.collecting_data_starting_time = rospy.Time.now()
        self.control_client.send_goal(goal)

        if time_gap_send_cmd_again > 1e-5:
            while True:
                rospy.sleep(time_gap_send_cmd_again)
                time_elapsed = (rospy.Time.now() -
                                self.collecting_data_starting_time).to_sec()
                if completion_time <= time_elapsed:
                    break

                goal = FollowJointTrajectoryGoal()
                goal.trajectory.joint_names = self.joint_names
                point = JointTrajectoryPoint()
                point.positions = pos2
                point.velocities = vel2
                point.time_from_start = rospy.Time(
                    completion_time - time_elapsed)
                goal.trajectory.points.append(point)
                self.control_client.send_goal(goal)
                print "Send"
        rospy.sleep(2.0)
        self.collecting_data_starting_time = None

        if not save:
            return

        # https://pythonspot.com/matplotlib-scatterplot/
        data = ((self.times, [x[0] for x in self.jtposs]),
                (self.times, [x[0] for x in self.jtvels]))
        colors = ("red", "green")
        groups = ("pos", "vel")
        fig = plt.figure()
        ax = fig.add_subplot(1, 1, 1)
        for i in range(len(data)):
            x, y = data[i]
            color = colors[i]
            group = groups[i]
            ax.plot(x, y, alpha=0.5, c=color, linewidth=4, label=group)
        plt.legend(loc=2)

        if self.vel_control:
            plt_name = "vel_control_time_gap_send_cmd_again_" + str(
                time_gap_send_cmd_again)
        else:
            plt_name = "pos_control_time_gap_send_cmd_again_" + str(
                time_gap_send_cmd_again)
        plt.title(plt_name)

        # plt.show()
        config_path = find_in_workspaces(
            search_dirs=['share'],
            project="ur_modern_driver",
            path="tests",
            first_match_only=True)
        path = os.path.join(config_path[0],
                            "test_modern_driver_ros_controller_plots",
                            plt_name + ".pdf")
        print path
        plt.savefig(path)

    def test_stop(self, save=False):
        freq = 500
        rate = rospy.Rate(freq)

        goal_pose = copy.deepcopy(self.cur_jtpos)
        goal_pose[0] -= 0.2
        pos2 = list(goal_pose)
        vel2 = [0.0] * self.num_dofs

        completion_time = 1.0

        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = pos2
        point.velocities = vel2
        point.time_from_start = rospy.Time(completion_time)
        goal.trajectory.points.append(point)

        self.collecting_data_starting_time = rospy.Time.now()
        self.control_client.send_goal(goal)

        rospy.sleep(0.3)

        # https://groups.google.com/forum/#!topic/swri-ros-pkg-dev/qo9pu4PbEJY
        # The correct way to stop the robot while executing a trajectory is not to send another goal with an empty trajectory. Rather you should cancel your previous goal request with a call to ac_->cancelGoal(); .
        self.control_client.cancel_all_goals()

        rospy.sleep(2.0)
        self.collecting_data_starting_time = None

        if not save:
            return

        # https://pythonspot.com/matplotlib-scatterplot/
        data = ((self.times, [x[0] for x in self.jtposs]),
                (self.times, [x[0] for x in self.jtvels]))
        colors = ("red", "green")
        groups = ("pos", "vel")
        fig = plt.figure()
        ax = fig.add_subplot(1, 1, 1)
        for i in range(len(data)):
            x, y = data[i]
            color = colors[i]
            group = groups[i]
            ax.plot(x, y, alpha=0.5, c=color, linewidth=4, label=group)
        plt.legend(loc=2)

        if self.vel_control:
            plt_name = "vel_control_stop"
        else:
            plt_name = "pos_control_stop"
        plt.title(plt_name)

        # plt.show()
        config_path = find_in_workspaces(
            search_dirs=['share'],
            project="ur_modern_driver",
            path="tests",
            first_match_only=True)
        path = os.path.join(config_path[0],
                            "test_modern_driver_ros_controller_plots",
                            plt_name + ".pdf")
        print path
        plt.savefig(path)

    def cur_jtstate_callback(self, data):
        self.cur_jtpos = [0.0] * self.num_dofs
        self.cur_jtvel = [0.0] * self.num_dofs
        for i, jn in enumerate(self.joint_names):
            index = data.name.index(jn)
            self.cur_jtpos[i] = data.position[index]
            self.cur_jtvel[i] = data.velocity[index]
        if self.collecting_data_starting_time is not None:
            self.times.append((rospy.Time.now() -
                               self.collecting_data_starting_time).to_sec())
            self.jtposs.append(self.cur_jtpos)
            self.jtvels.append(self.cur_jtvel)


if __name__ == "__main__":
    rospy.init_node('test_vel_controller', anonymous=True)
    while not rospy.Time.now():
        pass

    #  r = Robot(vel_control=True)
    #  r.test_control(time_gap_send_cmd_again=-1, save=False)
    #  r.test_control(time_gap_send_cmd_again=0.3, save=False)
    #  r.test_stop(save=False)

    #  r = Robot(vel_control=False)
    #  r.test_control(time_gap_send_cmd_again=-1, save=False)
    #  r.test_control(time_gap_send_cmd_again=0.3, save=False)
    #  r.test_stop(save=False)
